using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Core.Units;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Subsystems.Navigation;

[Subsystem("NavigationSubsystem", Disabled = false)]
public class NavigationSubsystem : SubsystemBase
{
    private readonly AStarConfig _config = new();
    private readonly PurePursuit _pursuit = new();

    private VectornavReport? _prevPosition;
    private VectornavReport? _position;
    private float? _robotThetaRad;

    private double? _waypointLat;
    private double? _waypointLng;
    private float? _waypointHeadingRad;

    private ulong _autonomousStartTime = 0;
    private bool _waypointsActive = false;

    public IReadOnlyList<(int x, int y)>? LastGridPath { get; private set; }
    public IReadOnlyList<(float x, float y)>? LastLocalPath { get; private set; }

    public override Task Init(CancellationToken token)
    {
        SubscribeImage("combined_inflated", OnInflatedImageReceived, token);
        SubscribeMessage<Waypoint>(MessageType.Waypoint, OnWaypointReceived, token);

        SetOperatingState(SubsystemState.Operating);
        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        if (updated.Mode == RobotModeEnum.Autonomous && updated.MotionAllowed)
        {
            if (_autonomousStartTime == 0)
            {
                _autonomousStartTime = TimeUtils.Now();
                _waypointsActive = false;
                Logger.LogInformation("Navigation entered autonomous — waypoints active in {DelayMs}ms", _config.WaypointDelayMs);
            }
        }
        else
        {
            _autonomousStartTime = 0;
            _waypointsActive = false;
            _waypointLat = null;
            _waypointLng = null;
            _waypointHeadingRad = null;
            _prevPosition = null;
            _robotThetaRad = null;
        }

        return Task.CompletedTask;
    }

    private Task OnWaypointReceived(Waypoint msg, CancellationToken token)
    {
        _waypointLat = msg.Latitude;
        _waypointLng = msg.Longitude;
        Logger.LogDebug("Waypoint received — lat={Lat} lng={Lng}", _waypointLat, _waypointLng);

        if (Position != null)
        {
            UpdateWaypointHeading(Position);
        }

        return Task.CompletedTask;
    }

    public override void OnPositionChanged(RobotPosition position)
    {
        if (position.Heading == null)
        {
            return;
        }

        _robotThetaRad = (float)position.Heading.Value.To(AngleUnit.Radians);
        _prevPosition = _position;
        if (!_waypointsActive && _autonomousStartTime != 0)
        {
            if ((TimeUtils.Now() - _autonomousStartTime) >= _config.WaypointDelayMs)
            {
                _waypointsActive = true;
                Logger.LogInformation("Waypoint delay elapsed — waypoints now influencing path planning");
            }
        }

        UpdateWaypointHeading(position);
    }

    private void UpdateWaypointHeading(RobotPosition position)
    {
        if (!_waypointLat.HasValue || !_waypointLng.HasValue) return;

        var robotPos = new LatLng(position.Coordinates.Latitude, position.Coordinates.Longitude);
        var waypointPos = new LatLng(_waypointLat.Value, _waypointLng.Value);
        var headingToWaypoint = GeoUtils.HeadingToPosition(robotPos, waypointPos);
        _waypointHeadingRad = (float)headingToWaypoint.To(AngleUnit.Radians);
    }

    private Task OnInflatedImageReceived(ImageFrame frame, CancellationToken token)
    {
        try
        {
            ProcessFrame(frame);
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Navigation processing failed");
        }
        return Task.CompletedTask;
    }

    private void ProcessFrame(ImageFrame frame)
    {
        int W = _config.GridWidth;
        int H = _config.GridHeight;

        byte[] costMap;

        if (_config.UseOnlyWaypoints)
        {
            // Ignore vision entirely — flat cost map, steer purely by waypoint heading
            costMap = new byte[W * H];
        }
        else
        {
            using var mat = CvUtils.AsMat(frame);
            using var resized = new Mat();
            if (mat.Width != W || mat.Height != H)
                Cv2.Resize(mat, resized, new Size(W, H), interpolation: InterpolationFlags.Nearest);

            var src = resized.Empty() ? mat : resized;

            using var gray = new Mat();
            if (src.Channels() > 1)
                Cv2.CvtColor(src, gray, ColorConversionCodes.BGR2GRAY);
            else
                src.CopyTo(gray);

            costMap = new byte[W * H];
            for (int y = 0; y < H; y++)
                for (int x = 0; x < W; x++)
                    costMap[x + W * y] = gray.At<byte>(y, x);
        }

        float? activeWaypointHeading = _waypointsActive ? _waypointHeadingRad : null;
        float? activeRobotTheta = _robotThetaRad;

        var gridPath = AStarPlanner.FindPath(costMap, _config, activeWaypointHeading, activeRobotTheta);
        if (gridPath is null || gridPath.Count == 0)
        {
            Logger.LogWarning("A* found no path — open space may be fully blocked");
            return;
        }

        LastGridPath = gridPath;

        var localPath = gridPath
            .Select(cell => AStarPlanner.GridToLocal(cell, _config))
            .ToList();
        LastLocalPath = localPath;

        _pursuit.SetPoints(localPath);

        var goal = gridPath[^1];

        using var debugMat = AStarPlanner.BuildDebugImage(costMap, gridPath, goal, _config, displayScale: 4);
        PublishImage(debugMat, "nav_astar_debug");

        if (!_config.UseOnlyWaypoints)
        {
            using var mat = CvUtils.AsMat(frame);
            using var overlayMat = OverlayPathOnSource(mat, gridPath, goal);
            PublishImage(overlayMat, "nav_path_overlay");
        }
    }

    private static Mat OverlayPathOnSource(
        Mat src,
        List<(int x, int y)> path,
        (int x, int y) goal)
    {
        var overlay = src.Channels() == 1
            ? src.CvtColor(ColorConversionCodes.GRAY2BGR)
            : src.Clone();

        foreach (var (px, py) in path)
            Cv2.Circle(overlay, new Point(px, py), 1, new Scalar(0, 255, 0), -1);

        Cv2.Circle(overlay, new Point(goal.x, goal.y), 3, new Scalar(255, 80, 0), -1);
        Cv2.Circle(overlay, new Point(overlay.Width / 2, overlay.Height - 2), 3,
            new Scalar(0, 0, 255), -1);

        return overlay;
    }

    private void PublishImage(Mat mat, string name)
    {
        var bytes = CvUtils.FromMat(mat);
        var imgFrame = MessageConstructor.CreateImageFrame(
            (uint)mat.Width, (uint)mat.Height, name, bytes);
        EventBus.Instance.Publish(new MessageWrapperEvent(
            MessageWrapper.From(MessageType.ImageFrame, imgFrame.ByteBuffer.ToFullArray())));
    }
}