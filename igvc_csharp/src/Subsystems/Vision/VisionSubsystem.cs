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
                        new(centerMat.Width * 0.65 - 50, centerMat.Height * 0.80),
                        new(centerMat.Width * 0.29 + 50, centerMat.Height * 0.80),
                    }
                };
                Cv2.FillPoly(centerMat, points, Scalar.Black);

                // Draw the region of disinterest as described above
                var centerMatClone = centerMat.Clone();
                Cv2.Polylines(centerMatClone, points, true, Scalar.Red, 2);

                // Publish debug image
                var hsvBytes = CvUtils.FromMat(centerMatClone);
                var hsvFrame = MessageConstructor.CreateImageFrame(
                    (uint)centerMatClone.Width,
                    (uint)centerMatClone.Height,
                    "combined_filtered",
                    hsvBytes
                );
                EventBus.Instance.Publish(new MessageWrapperEvent(
                    MessageWrapper.From(MessageType.ImageFrame, hsvFrame.ByteBuffer.ToFullArray())
                ));
                centerMatClone.Dispose();

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