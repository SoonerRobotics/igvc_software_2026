using igvc_csharp.Core;
using igvc_csharp.Core.Units;
using igvc_csharp.Events;
using igvc_csharp.src.selfdrive.actions;
using igvc_csharp.src.subsystems.selfdrive;
using igvc_csharp.src.Utils;
using igvc_csharp.Utils;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.src.Subsystems.selfdrive.actions;

public class LaneKeepAction(
    SelfdriveObstacles obstacle,
    Distance distanceToStop,
    ulong timeoutMs = 0
) : ISelfdriveAction
{
    private readonly ILogger _logger = Logging.From<LaneKeepAction>();
    private ulong _startTime;
    private bool _isInit;

    /// <summary>
    /// Minimum pixel distance the robot must maintain from a white lane line.
    /// Measured from the horizontal center of the image outward to each side.
    /// </summary>
    private const int MinLaneDistancePx = 150;

    public bool IsInit() => _isInit;

    public void Init(SelfdriveContext context)
    {
        _startTime = TimeUtils.Now();
        _isInit = true;
    }

    public void Run(SelfdriveContext context)
    {
        Mat? frame;
        lock (context)
        {
            frame = context.LastFilteredFrame?.Clone();
        }

        if (frame == null)
        {
            _logger.LogWarning("No filtered frame available, skipping lane keep tick.");
            return;
        }

        using (frame)
        {
            // Ensure single-channel grayscale
            if (frame.Channels() > 1)
            {
                var gray = new Mat();
                Cv2.CvtColor(frame, gray, ColorConversionCodes.BGR2GRAY);
                frame.Dispose();
                frame = gray;
            }

            if (!frame.GetArray(out byte[] rawPixels))
            {
                _logger.LogError("Couldn't get raw pixel data.");
                return;
            }

            int cols = frame.Cols;
            int rows = frame.Rows;
            int centerX = cols / 2;

            // Scan a horizontal slice near the bottom of the image (90% down),
            // matching the original feeler scan row.
            int scanRow = (int)(rows * 0.9);

            int leftLaneX = FindNearestWhitePixel(rawPixels, cols, scanRow, centerX, scanDirection: -1);
            int rightLaneX = FindNearestWhitePixel(rawPixels, cols, scanRow, centerX, scanDirection: +1);

            // Distances from center to each detected lane (positive = lane is further out)
            int distLeft = leftLaneX >= 0 ? centerX - leftLaneX : int.MaxValue;
            int distRight = rightLaneX >= 0 ? rightLaneX - centerX : int.MaxValue;

            _logger.LogDebug(
                // "Lane distances — left: {L}px (lane@{LX}), right: {R}px (lane@{RX}), center: {C}",
                distLeft == int.MaxValue ? "∞" : distLeft.ToString(), leftLaneX,
                distRight == int.MaxValue ? "∞" : distRight.ToString(), rightLaneX,
                centerX);

            // Correction: positive → steer right, negative → steer left.
            // If we're too close to the left lane, push right (positive).
            // If we're too close to the right lane, push left (negative).
            double correction = 0.0;

            bool tooCloseLeft = distLeft != int.MaxValue && distLeft < MinLaneDistancePx;
            bool tooCloseRight = distRight != int.MaxValue && distRight < MinLaneDistancePx;

            if (tooCloseLeft && tooCloseRight)
            {
                // Squeezed between both lanes — steer toward whichever side has more room.
                correction = (distRight - distLeft);
                // _logger.LogDebug("Both lanes close, balancing. Correction: {C:F3}", correction);
            }
            else if (tooCloseLeft)
            {
                correction = +(MinLaneDistancePx - distLeft);
                // _logger.LogDebug("Too close to left lane by {D}px, correcting right.", MinLaneDistancePx - distLeft);
            }
            else if (tooCloseRight)
            {
                correction = -(MinLaneDistancePx - distRight);
                // _logger.LogDebug("Too close to right lane by {D}px, correcting left.", MinLaneDistancePx - distRight);
            }

            if (BaseRobot.Instance?.State.MotionAllowed ?? false)
            {
                // Scale correction into angular velocity, clamped to TurnSpeed.
                double maxTurn = SelfdriveSubsystem.TurnSpeed.To(AngularVelocityUnit.RadiansPerSecond);
                double turnScale = maxTurn / MinLaneDistancePx;   // 1 px of error → proportional rad/s
                double turningVelocity = Math.Clamp(correction * turnScale, -maxTurn, maxTurn);

                // _logger.LogDebug("LaneKeep turning velocity: {V:F4} rad/s", turningVelocity);

                context.Canbus.MotorControl.SetVelocities(
                    SelfdriveSubsystem.ForwardSpeed.To(LinearVelocityUnit.MetersPerSecond),
                    0,
                    turningVelocity);
            }
            else
            {
                _logger.LogDebug("Motion not allowed, holding stop.");
                context.Canbus.MotorControl.SetVelocities(0, 0, 0);
            }
        }
    }

    /// <summary>
    /// Scans outward from <paramref name="startX"/> along <paramref name="row"/> and returns
    /// the X coordinate of the first white pixel found, or -1 if none is found before the edge.
    /// </summary>
    /// <param name="pixels">Raw grayscale pixel data (1 byte per pixel).</param>
    /// <param name="cols">Image width in pixels.</param>
    /// <param name="row">Row index to scan.</param>
    /// <param name="startX">Starting X position (image center).</param>
    /// <param name="scanDirection">-1 to scan left, +1 to scan right.</param>
    private static int FindNearestWhitePixel(
        byte[] pixels, int cols, int row, int startX, int scanDirection)
    {
        const byte WhiteThreshold = 200;
        int rowOffset = row * cols;

        for (int x = startX; x >= 0 && x < cols; x += scanDirection)
        {
            if (pixels[rowOffset + x] >= WhiteThreshold)
                return x;
        }

        return -1; // No white pixel found on this side
    }

    public bool IsFinished(SelfdriveContext context)
    {
        if (!_isInit) return false;
        if (timeoutMs > 0 && (TimeUtils.Now() - _startTime) > timeoutMs) return true;

        if (context.DetectionTracker.TryGetConfirmed(obstacle.ToString().ToLower(), out var detection))
        {
            _logger.LogInformation(
                "Person in view — distance: {Dist:F3}m/{DistY:F3}/{DistZ:F3}, stop threshold: {Stop:F3}m, will stop: {Stop}",
                detection.x,
                detection.y,
                detection.z,
                distanceToStop.To(DistanceUnit.Meters),
                detection.z <= distanceToStop.To(DistanceUnit.Meters));

            if (Math.Abs(detection.z) <= distanceToStop.To(DistanceUnit.Meters))
                return true;
        }

        return false;
    }

    public void End(SelfdriveContext context)
    {
        context.Canbus.MotorControl.SetVelocities(0, 0, 0);
    }
}