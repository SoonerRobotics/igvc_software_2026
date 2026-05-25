using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Subsystems.Vision.Filters;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

using VisionConfig = igvc_csharp.Configuration.VisionSubsystem;

namespace igvc_csharp.Subsystems.Vision;

[Subsystem("VisionSubsystem", Disabled = false)]
public class VisionSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    private readonly List<IFilter> _leftFilters = [];
    private readonly List<IFilter> _rightFilters = [];

    // Guards against concurrent filter rebuilds (config change vs state change)
    private readonly Lock _filterLock = new();

    // Config paths that require a filter rebuild when changed
    private static readonly HashSet<string> _visionConfigPaths =
    [
        "vision.ground_threshold",
        "vision.yellow_threshold",
        "vision.blur_radius",
        "vision.blur_strength",
    ];

    private readonly Channel<ImageFrame> _leftChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    private readonly Channel<ImageFrame> _rightChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    public override Task Init(CancellationToken token)
    {
        RebuildFilters(BaseRobot.Instance.State);

        SubscribeImage("left", OnLeftImageReceived, token);
        SubscribeImage("right", OnRightImageReceived, token);

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
            return Task.CompletedTask;

        Logger.LogInformation("Vision config changed ({Path}), rebuilding filters", e.Path);
        RebuildFilters(BaseRobot.Instance.State);
        return Task.CompletedTask;
    }

    private void RebuildFilters(RobotState state)
    {
        lock (_filterLock)
        {
            _leftFilters.Clear();
            _rightFilters.Clear();
            AddFilters(state);
        }
    }

    private void AddFilters(RobotState newState)
    {
        // standard lane / obstacle detection
        _leftFilters.Add(new HsvFilter(VisionConfig.GroundThreshold, HsvFilter.OutputMode.WhiteForOutside));
        _rightFilters.Add(new HsvFilter(VisionConfig.GroundThreshold, HsvFilter.OutputMode.WhiteForOutside));

        if (newState.Mission == MissionEnum.Autonav)
        {
            _leftFilters.Insert(0, new BlurFilter(VisionConfig.BlurRadius, VisionConfig.BlurStrength, BlurFilter.BlurMethod.BoxBlur));
            _rightFilters.Insert(0, new BlurFilter(VisionConfig.BlurRadius, VisionConfig.BlurStrength, BlurFilter.BlurMethod.BoxBlur));

            _leftFilters.Add(new TopDownFilter(
                VisionConfig.leftSourcePoints,
                VisionConfig.leftDestPoints,
                new Size(640, 480)
            ));

            _rightFilters.Add(new TopDownFilter(
                VisionConfig.rightSourcePoints,
                VisionConfig.rightDestPoints,
                new Size(640, 480)
            ));
        }

        if (newState.Mission == MissionEnum.Selfdrive)
        {
            _leftFilters.Add(new LaneDetectionFilter());
            _rightFilters.Add(new LaneDetectionFilter());
        }
    }

    private async Task ImageProcessingTask(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                var leftTask = _leftChannel.Reader.ReadAsync(token).AsTask();
                var rightTask = _rightChannel.Reader.ReadAsync(token).AsTask();
                await Task.WhenAll(leftTask, rightTask);

                var leftFrame = leftTask.Result;
                var rightFrame = rightTask.Result;

                var leftMat = CvUtils.AsMat(leftFrame);
                var rightMat = CvUtils.AsMat(rightFrame);

                // Snapshot the filter lists under lock so a concurrent rebuild
                // doesn't modify them mid-pipeline.
                IFilter[] leftFilters;
                IFilter[] rightFilters;
                lock (_filterLock)
                {
                    leftFilters = [.. _leftFilters];
                    rightFilters = [.. _rightFilters];
                }

                leftMat = leftFilters.Aggregate(leftMat, (current, filter) => filter.Apply(current));
                rightMat = rightFilters.Aggregate(rightMat, (current, filter) => filter.Apply(current));

                // Ensure both mats are BGR before combining
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

                leftMat = EnsureBgr(leftMat);
                rightMat = EnsureBgr(rightMat);

                var combinedFiltered = new Mat();

                if (BaseRobot.Instance.State.Mission == MissionEnum.Selfdrive)
                {
                    combinedFiltered = CombineAndAnnotate(leftMat, rightMat, scale: 0.5);
                }
                else if (BaseRobot.Instance.State.Mission == MissionEnum.Autonav)
                {
                    Cv2.HConcat([leftMat, rightMat], combinedFiltered);
                }

                if (!combinedFiltered.Empty())
                {
                    var combinedFilteredBytes = CvUtils.FromMat(combinedFiltered);
                    var newFrame = MessageConstructor.CreateImageFrame(
                        (uint)combinedFiltered.Width,
                        (uint)combinedFiltered.Height,
                        "combined_filtered",
                        combinedFilteredBytes
                    );

                    EventBus.Instance.Publish(new MessageWrapperEvent(
                        MessageWrapper.From(MessageType.ImageFrame, newFrame.ByteBuffer.ToFullArray())));

                    var thresholdFilter = new ThresholdFilter();
                    var combinedThresholded = thresholdFilter.Apply(combinedFiltered);

                    var inflationFilter = new InflationFilter(kernelWidth: 31, kernelHeight: 31);
                    combinedThresholded = inflationFilter.Apply(combinedThresholded);

                    var combinedInflatedBytes = CvUtils.FromMat(combinedThresholded);
                    var inflatedFrame = MessageConstructor.CreateImageFrame(
                        (uint)combinedThresholded.Width,
                        (uint)combinedThresholded.Height,
                        "combined_inflated",
                        combinedInflatedBytes
                    );

                    EventBus.Instance.Publish(new MessageWrapperEvent(
                        MessageWrapper.From(MessageType.ImageFrame, inflatedFrame.ByteBuffer.ToFullArray())));

                    combinedThresholded.Dispose();
                }
                else
                {
                    Logger.LogWarning("No mission matched, skipping combined publish");
                }

                // Raw combined view for debug
                var leftRaw = CvUtils.AsMat(leftFrame);
                var rightRaw = CvUtils.AsMat(rightFrame);

                var leftFilter = new TopDownFilter(
                    VisionConfig.leftSourcePoints,
                    VisionConfig.leftDestPoints,
                    new Size(640, 480)
                );
                var rightFilter = new TopDownFilter(
                    VisionConfig.rightSourcePoints,
                    VisionConfig.rightDestPoints,
                    new Size(640, 480)
                );
                leftRaw = leftFilter.Apply(leftRaw);
                rightRaw = rightFilter.Apply(rightRaw);

                var combinedRaw = new Mat();
                Cv2.HConcat([leftRaw, rightRaw], combinedRaw);

                var combinedBytes = CvUtils.FromMat(combinedRaw);
                var rawFrame = MessageConstructor.CreateImageFrame(
                    (uint)combinedRaw.Width,
                    (uint)combinedRaw.Height,
                    "combined_view",
                    combinedBytes
                );

                EventBus.Instance.Publish(new MessageWrapperEvent(
                    MessageWrapper.From(MessageType.ImageFrame, rawFrame.ByteBuffer.ToFullArray())));

                leftMat.Dispose();
                rightMat.Dispose();
                combinedFiltered.Dispose();
                leftRaw.Dispose();
                rightRaw.Dispose();
                combinedRaw.Dispose();
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Vision processing task crashed");
        }
    }

    private static Mat CombineAndAnnotate(Mat left, Mat right, double scale = 0.5)
    {
        var scaledLeft = new Mat();
        var scaledRight = new Mat();
        Cv2.Resize(left, scaledLeft, new Size(0, 0), scale, scale, InterpolationFlags.Area);
        Cv2.Resize(right, scaledRight, new Size(0, 0), scale, scale, InterpolationFlags.Area);
        left = scaledLeft;
        right = scaledRight;

        using var leftYellow = new Mat();
        using var rightYellow = new Mat();
        Cv2.InRange(left, new Scalar(0, 254, 254), new Scalar(1, 255, 255), leftYellow);
        Cv2.InRange(right, new Scalar(0, 254, 254), new Scalar(1, 255, 255), rightYellow);

        var leftYellowCount = Cv2.CountNonZero(leftYellow);
        var rightYellowCount = Cv2.CountNonZero(rightYellow);

        string laneLabel;
        if (leftYellowCount == 0 && rightYellowCount == 0)
            laneLabel = "Lane: UNKNOWN";
        else if (leftYellowCount > rightYellowCount * 1.5)
            laneLabel = "Lane: RIGHT";
        else if (rightYellowCount > leftYellowCount * 1.5)
            laneLabel = "Lane: LEFT";
        else
            laneLabel = "Lane: CENTER";

        var divider = new Mat(left.Height, 4, MatType.CV_8UC3, new Scalar(80, 80, 80));
        var leftLabeled = left.Clone();
        var rightLabeled = right.Clone();
        Cv2.PutText(leftLabeled, "LEFT VIEW", new Point(10, 30), HersheyFonts.HersheySimplex, 0.8, new Scalar(200, 200, 200), 2);
        Cv2.PutText(rightLabeled, "RIGHT VIEW", new Point(10, 30), HersheyFonts.HersheySimplex, 0.8, new Scalar(200, 200, 200), 2);

        var combined = new Mat();
        Cv2.HConcat(new[] { leftLabeled, divider, rightLabeled }, combined);
        leftLabeled.Dispose();
        rightLabeled.Dispose();
        divider.Dispose();
        scaledLeft.Dispose();
        scaledRight.Dispose();

        var bannerH = 50;
        var (bannerColor, textColor) = laneLabel switch
        {
            "Lane: LEFT" => (new Scalar(255, 100, 0), new Scalar(255, 255, 255)),
            "Lane: RIGHT" => (new Scalar(0, 100, 255), new Scalar(255, 255, 255)),
            "Lane: CENTER" => (new Scalar(0, 200, 80), new Scalar(0, 0, 0)),
            _ => (new Scalar(40, 40, 40), new Scalar(160, 160, 160)),
        };

        Cv2.Rectangle(combined, new Rect(0, 0, combined.Width, bannerH), bannerColor, thickness: -1);

        var textSize = Cv2.GetTextSize(laneLabel, HersheyFonts.HersheySimplex, 1.0, 2, out _);
        var textX = combined.Width / 2 - textSize.Width / 2;
        Cv2.Rectangle(combined, new Rect(textX - 10, 8, textSize.Width + 20, bannerH - 16), new Scalar(0, 0, 0), thickness: -1);
        Cv2.PutText(combined, laneLabel, new Point(textX, 35), HersheyFonts.HersheySimplex, 1.0, textColor, thickness: 2);

        return combined;
    }

    private Task OnLeftImageReceived(ImageFrame frame, CancellationToken token)
    {
        _leftChannel.Writer.TryWrite(frame);
        return Task.CompletedTask;
    }

    private Task OnRightImageReceived(ImageFrame frame, CancellationToken token)
    {
        _rightChannel.Writer.TryWrite(frame);
        return Task.CompletedTask;
    }
}