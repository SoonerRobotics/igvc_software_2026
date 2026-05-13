using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Subsystems.Vision.Filters;
using igvc_csharp.Utils;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision;

[Subsystem("VisionSubsystem", DependsOn = [typeof(CanbusSubsystem)], Disabled = false)]
public class VisionSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    private readonly List<IFilter> _filters = [];

    private readonly OpenCvImageWindow _window = new("Vision Output");

    private readonly Channel<ImageFrame> _leftChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true, SingleWriter = false, FullMode = BoundedChannelFullMode.DropOldest
    });

    private readonly Channel<ImageFrame> _rightChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true, SingleWriter = false, FullMode = BoundedChannelFullMode.DropOldest
    });

    public override Task Init(CancellationToken token)
    {
        AddFilters();

        SubscribeImage("left_view",  OnLeftImageReceived,  token);
        SubscribeImage("right_view", OnRightImageReceived, token);

        _ = Task.Factory.StartNew(
            () => ImageProcessingTask(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        SetOperatingState(SubsystemState.Operating);
        return Task.CompletedTask;
    }

    private void AddFilters()
    {
        _filters.Add(new LaneDetectionFilter());
    }

    private async Task ImageProcessingTask(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                // Wait for both frames
                var leftFrame  = await _leftChannel.Reader.ReadAsync(token);
                var rightFrame = await _rightChannel.Reader.ReadAsync(token);

                // Apply lane detection filter to each view
                var leftMat  = CvUtils.AsMat(leftFrame);
                var rightMat = CvUtils.AsMat(rightFrame);

                leftMat  = _filters.Aggregate(leftMat,  (current, filter) => filter.Apply(current));
                rightMat = _filters.Aggregate(rightMat, (current, filter) => filter.Apply(current));

                // Combine and determine lane position
                var combined = CombineAndAnnotate(leftMat, rightMat);

                _window.EnqueueJpeg(CvUtils.FromMat(combined));

                leftMat.Dispose();
                rightMat.Dispose();
                combined.Dispose();
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Vision processing task crashed");
        }
    }
    
    private static Mat CombineAndAnnotate(Mat left, Mat right)
    {
        using var leftYellow  = new Mat();
        using var rightYellow = new Mat();
        Cv2.InRange(left,  new Scalar(0, 254, 254), new Scalar(1, 255, 255), leftYellow);
        Cv2.InRange(right, new Scalar(0, 254, 254), new Scalar(1, 255, 255), rightYellow);

        var leftYellowCount  = Cv2.CountNonZero(leftYellow);
        var rightYellowCount = Cv2.CountNonZero(rightYellow);

        // Determine lane position based on which side has more yellow
        // more yellow on left -> robot is in right lane, and vice versa
        string laneLabel;

        if (leftYellowCount == 0 && rightYellowCount == 0)
        {
            laneLabel = "Lane: Unknown";
        }
        else if (leftYellowCount > rightYellowCount * 1.5)
        {
            laneLabel = "Lane: RIGHT";
        }
        else if (rightYellowCount > leftYellowCount * 1.5)
        {
            laneLabel = "Lane: LEFT";
        }
        else
        {
            laneLabel = "Lane: CENTER";
        }

        // Stack together
        var dividerWidth = 4;
        var divider = new Mat(left.Height, dividerWidth, MatType.CV_8UC3, new Scalar(80, 80, 80));

        // Labels
        var leftLabeled  = left.Clone();
        var rightLabeled = right.Clone();
        Cv2.PutText(leftLabeled,  "LEFT VIEW",  new Point(10, 30), HersheyFonts.HersheySimplex, 0.8, new Scalar(200, 200, 200), 2);
        Cv2.PutText(rightLabeled, "RIGHT VIEW", new Point(10, 30), HersheyFonts.HersheySimplex, 0.8, new Scalar(200, 200, 200), 2);

        // Combined
        var combined = new Mat();
        Cv2.HConcat(new[] { leftLabeled, divider, rightLabeled }, combined);
        leftLabeled.Dispose();
        rightLabeled.Dispose();
        divider.Dispose();

        // Banner
        var bannerH = 50;
        Cv2.Rectangle(combined, new Rect(0, 0, combined.Width, bannerH), new Scalar(0, 255, 0), thickness: -1);
        Cv2.PutText(
            combined,
            laneLabel,
            new Point(combined.Width / 2 - 120, 35),
            HersheyFonts.HersheySimplex,
            1.2,
            new Scalar(0, 0, 0),
            thickness: 2
        );

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