using System.Threading.Channels;
using Google.FlatBuffers;
using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Subsystems.Vision.Filters;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Messages.Arc;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

using VisionConfig = igvc_csharp.Configuration.VisionSubsystem;

namespace igvc_csharp.Subsystems.Vision;

[Subsystem("VisionSubsystem", Disabled = false)]
public class VisionSubsystem() : SubsystemBase
{
    private readonly List<IFilter> _centerFilters = [];
    private readonly Lock _filterLock = new();
    private static readonly HashSet<string> _visionConfigPaths =
    [
        "vision.ground_threshold",
        "vision.yellow_threshold",
        "vision.blur_radius",
        "vision.blur_strength",
    ];

    private readonly Channel<ImageFrame> mCenterChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    public override Task Init(CancellationToken token)
    {
        RebuildFilters(BaseRobot.Instance?.State);

        SubscribeImage("center", OnCenterImageReceived, token);
        Subscribe<ConfigChangedEvent>(OnConfigChanged, token);

        _ = Task.Factory.StartNew(
            () => ImageProcessingTask(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        SetOperatingState(SubsystemState.Operating);
        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        SetOperatingState(SubsystemState.Starting);
        RebuildFilters(updated);
        SetOperatingState(SubsystemState.Ready);
        return Task.CompletedTask;
    }

    private Task OnConfigChanged(ConfigChangedEvent e, CancellationToken token)
    {
        if (!_visionConfigPaths.Contains(e.Path))
        {
            return Task.CompletedTask;
        }

        Logger.LogInformation("Vision config changed ({Path}), rebuilding filters", e.Path);
        RebuildFilters(BaseRobot.Instance?.State);
        return Task.CompletedTask;
    }

    private void RebuildFilters(RobotState? state)
    {
        if (state == null)
        {
            return;
        }

        lock (_filterLock)
        {
            _centerFilters.Clear();
            AddFilters(state);
        }
    }

    private void AddFilters(RobotState newState)
    {
    }

    private async Task ImageProcessingTask(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                // Apply HSV and Blur
                var centerFrame = await mCenterChannel.Reader.ReadAsync(token).AsTask();
                var centerMat = CvUtils.AsMat(centerFrame);
                var blurFilter = new BlurFilter(VisionConfig.BlurRadius, VisionConfig.BlurStrength);
                var hsvFilter = new HsvFilter(
                    VisionConfig.GroundThreshold,
                    HsvFilter.OutputMode.WhiteForOutside
                );
                centerMat = blurFilter.Apply(centerMat);
                centerMat = hsvFilter.Apply(centerMat);

                // Apply ROI
                var points = new[]
                {
                    new Point[]
                    {
                        new(centerMat.Width * 0.29, centerMat.Height),
                        new(centerMat.Width * 0.68, centerMat.Height),
                        new(centerMat.Width * 0.65 - 50, centerMat.Height * 0.70),
                        new(centerMat.Width * 0.29 + 50, centerMat.Height * 0.70),
                    }
                };
                Cv2.FillPoly(centerMat, points, Scalar.Black);

                // Perform top down transformation
                var topDownInput = new[]
                {
                    new Point2f(260, 200), //TL
                    new Point2f(380, 200), //TR
                    new Point2f(120, 480), //BL
                    new Point2f(520, 480), //BR
                };

                var topDownOutput = new[]
                {
                    new Point2f(240, 0),
                    new Point2f(380, 0),
                    new Point2f(240, 480),
                    new Point2f(380, 480),
                };

                // Draw the region of disinterest and perspective transform corners
                var debugMat = centerMat.Clone();
                // Convert to BGR so colored overlays are visible (HsvFilter output is grayscale)
                if (debugMat.Channels() == 1)
                {
                    var tmp = new Mat();
                    Cv2.CvtColor(debugMat, tmp, ColorConversionCodes.GRAY2BGR);
                    debugMat.Dispose();
                    debugMat = tmp;
                }
                Cv2.Polylines(debugMat, points, true, Scalar.Red, 2);

                // Draw topDownInput quadrilateral (green) to verify perspective source coords
                var inputDebugPoly = new[]
                {
                    new Point[]
                    {
                        new((int)topDownInput[0].X, (int)topDownInput[0].Y),
                        new((int)topDownInput[1].X, (int)topDownInput[1].Y),
                        new((int)topDownInput[3].X, (int)topDownInput[3].Y),
                        new((int)topDownInput[2].X, (int)topDownInput[2].Y),
                    }
                };
                Cv2.Polylines(debugMat, inputDebugPoly, true, new Scalar(0, 255, 0), 2);
                string[] cornerLabels = ["TL", "TR", "BL", "BR"];
                Point2f[] labelOrder = [topDownInput[0], topDownInput[1], topDownInput[2], topDownInput[3]];
                for (int i = 0; i < labelOrder.Length; i++)
                {
                    var pt = new Point((int)labelOrder[i].X, (int)labelOrder[i].Y);
                    Cv2.Circle(debugMat, pt, 6, new Scalar(0, 255, 255), -1);
                    Cv2.PutText(debugMat, cornerLabels[i], new Point(pt.X + 8, pt.Y + 8),
                        HersheyFonts.HersheySimplex, 0.5, new Scalar(0, 255, 255), 1);
                }

                // Publish debug image
                var hsvBytes = CvUtils.FromMat(debugMat);
                var hsvFrame = MessageConstructor.CreateImageFrame(
                    (uint)debugMat.Width,
                    (uint)debugMat.Height,
                    "combined_filtered",
                    hsvBytes
                );
                EventBus.Instance.Publish(new MessageWrapperEvent(
                    MessageWrapper.From(MessageType.ImageFrame, hsvFrame.ByteBuffer.ToFullArray())
                ));
                debugMat.Dispose();

                var topDownFilter = new TopDownFilter(
                    topDownInput, topDownOutput, new Size(640, 480)
                );

                centerMat = topDownFilter.Apply(centerMat);

                // Inflate the image
                var inflationFilter = new InflationFilter();
                centerMat = inflationFilter.Apply(centerMat);

                static Mat EnsureBgr(Mat m)
                {
                    if (m.Channels() == 1)
                    {
                        var bgr = new Mat();
                        Cv2.CvtColor(m, bgr, ColorConversionCodes.GRAY2BGR);
                        m.Dispose();
                        return bgr;
                    }
                    return m;
                }

                // Publish image
                centerMat = EnsureBgr(centerMat);
                var combinedInflatedBytes = CvUtils.FromMat(centerMat);
                var inflatedFrame = MessageConstructor.CreateImageFrame(
                    (uint)centerMat.Width,
                    (uint)centerMat.Height,
                    "combined_inflated",
                    combinedInflatedBytes
                );

                EventBus.Instance.Publish(new MessageWrapperEvent(
                    MessageWrapper.From(MessageType.ImageFrame, inflatedFrame.ByteBuffer.ToFullArray())
                ));
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Vision processing task crashed");
        }
    }

    private Task OnCenterImageReceived(ImageFrame frame, CancellationToken token)
    {
        mCenterChannel.Writer.TryWrite(frame);
        return Task.CompletedTask;
    }
}