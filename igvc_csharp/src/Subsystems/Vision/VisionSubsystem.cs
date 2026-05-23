using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Subsystems.Vision.Filters;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision;

[Subsystem("VisionSubsystem", Disabled = false)]
public class VisionSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    private readonly List<IFilter> _leftFilters = [];
    private readonly List<IFilter> _rightFilters = [];

    // private readonly OpenCvImageWindow _window = new("Vision Output");

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
        AddFilters(BaseRobot.Instance.State);

        SubscribeImage("left", OnLeftImageReceived, token);
        SubscribeImage("right", OnRightImageReceived, token);

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

        _leftFilters.Clear();
        _rightFilters.Clear();

        AddFilters(updated);

        if (updated.Mission == MissionEnum.Autonav)
        {
            //TODO
        }
        else if (updated.Mission == MissionEnum.Selfdrive)
        {
            //TODO
        }

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    private void AddFilters(RobotState newState)
    {
        // standard lane / obstacle detection
        _leftFilters.Add(new HsvFilter(Configuration.VisionSubsystem.GroundThreshold, HsvFilter.OutputMode.WhiteForRange));
        _rightFilters.Add(new HsvFilter(Configuration.VisionSubsystem.GroundThreshold, HsvFilter.OutputMode.WhiteForRange));

        if (newState.Mission == MissionEnum.Autonav)
        {
            // insert as the first filter (don't want to blur if we're doing SelfDrive)
            _leftFilters.Insert(0, new BlurFilter(Configuration.VisionSubsystem.BlurRadius, Configuration.VisionSubsystem.BlurStrength, BlurFilter.BlurMethod.BoxBlur));
            _rightFilters.Insert(0, new BlurFilter(Configuration.VisionSubsystem.BlurRadius, Configuration.VisionSubsystem.BlurStrength, BlurFilter.BlurMethod.BoxBlur));

            //TODO: make region of disinterest configurable, although I don't think we'll actually need it...
            // _leftFilters.Add(new RegionFilter([
            //     new Point(0, 0), new Point(100, 100), new Point(50, 50)]
            // ));

            // _rightFilters.Add(new RegionFilter([
            //     new Point(0, 0), new Point(100, 100), new Point(50, 50)]
            // ));

            _leftFilters.Add(new TopDownFilter(
                Configuration.VisionSubsystem.leftSourcePoints,
                Configuration.VisionSubsystem.leftDestPoints,
                new Size(640, 480)
                // Configuration.AStarConfig.UseAStar ? new Size(80, 80) : new Size(640, 480) //FIXME don't hard-code resolutions / image sizes
            ));

            _rightFilters.Add(new TopDownFilter(
                Configuration.VisionSubsystem.rightSourcePoints,
                Configuration.VisionSubsystem.rightDestPoints,
                new Size(640, 480)
                // Configuration.AStarConfig.UseAStar ? new Size(80, 80) : new Size(640, 480) //FIXME don't hard-code resolutions / image sizes
            ));
        }

        // if (Configuration.AStarConfig.UseAStar)
        // {
        //     // don't need this for anything but A* (e.g. not for feelers)
        //     _leftFilters.Add(new InflationFilter());
        //     _rightFilters.Add(new InflationFilter());
        // }

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
                var leftFrame = await _leftChannel.Reader.ReadAsync(token);
                var rightFrame = await _rightChannel.Reader.ReadAsync(token);

                var leftMat = CvUtils.AsMat(leftFrame);
                var rightMat = CvUtils.AsMat(rightFrame);

                //TODO: parallelize these? I know we run super fast anyways but still I don't like it...
                leftMat = _leftFilters.Aggregate(leftMat, (current, filter) => filter.Apply(current));
                rightMat = _rightFilters.Aggregate(rightMat, (current, filter) => filter.Apply(current));

                var combinedFiltered = new Mat();

                if (BaseRobot.Instance.State.Mission == MissionEnum.Selfdrive)
                {
                    combinedFiltered = CombineAndAnnotate(leftMat, rightMat, scale: 0.5);

                    // _window.EnqueueJpeg(CvUtils.FromMat(combinedFiltered));
                }
                else if (BaseRobot.Instance.State.Mission == MissionEnum.Autonav)
                {
                    Cv2.HConcat([leftMat, rightMat], combinedFiltered);
                }

                // only publish the combined view of the filters, not left and right seperately
                var combinedFilteredBytes = CvUtils.FromMat(combinedFiltered);
                var newFrame = MessageConstructor.CreateImageFrame(
                    (uint)combinedFiltered.Width,
                    (uint)combinedFiltered.Height,
                    "combined_filtered",
                    combinedFilteredBytes
                );

                var wrappedFrame = MessageWrapper.From(
                    MessageType.ImageFrame,
                    newFrame.ByteBuffer.ToFullArray()
                );

                EventBus.Instance.Publish(new MessageWrapperEvent(wrappedFrame));

                // also publish the raw combined view for debug purposes
                var leftRaw = CvUtils.AsMat(leftFrame);
                var rightRaw = CvUtils.AsMat(rightFrame);

                var combinedRaw = new Mat();
                Cv2.HConcat([leftRaw, rightRaw], combinedRaw);

                var combinedBytes = CvUtils.FromMat(combinedRaw);
                var rawFrame = MessageConstructor.CreateImageFrame(
                    (uint)combinedRaw.Width,
                    (uint)combinedRaw.Height,
                    "combined_view",
                    combinedBytes
                );

                var wrappedRawFrame = MessageWrapper.From(
                    MessageType.ImageFrame,
                    rawFrame.ByteBuffer.ToFullArray()
                );

                EventBus.Instance.Publish(new MessageWrapperEvent(wrappedRawFrame));

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
        // Scale down before processing
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

        // Stack views with divider
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

        // Color-coded banner per lane state
        var bannerH = 50;
        var (bannerColor, textColor) = laneLabel switch
        {
            "Lane: LEFT" => (new Scalar(255, 100, 0), new Scalar(255, 255, 255)),  // orange
            "Lane: RIGHT" => (new Scalar(0, 100, 255), new Scalar(255, 255, 255)),  // blue
            "Lane: CENTER" => (new Scalar(0, 200, 80), new Scalar(0, 0, 0)),        // green
            _ => (new Scalar(40, 40, 40), new Scalar(160, 160, 160))   // dark grey
        };

        Cv2.Rectangle(combined, new Rect(0, 0, combined.Width, bannerH), bannerColor, thickness: -1);

        // Pill background behind text
        var textSize = Cv2.GetTextSize(laneLabel, HersheyFonts.HersheySimplex, 1.0, 2, out _);
        var textX = combined.Width / 2 - textSize.Width / 2;
        Cv2.Rectangle(
            combined,
            new Rect(textX - 10, 8, textSize.Width + 20, bannerH - 16),
            new Scalar(0, 0, 0),
            thickness: -1
        );
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