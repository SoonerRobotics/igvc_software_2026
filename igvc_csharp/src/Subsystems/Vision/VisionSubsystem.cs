using System.Diagnostics;
using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Core.Performance;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Vision.Filters;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision;

[Subsystem("VisionSubsystem", Disabled = false)]
public class VisionSubsystem : SubsystemBase
{
    private readonly List<IFilter> _filters = [];

    private readonly Channel<ImageFrame> _frameChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    // Calculates the average processing time over the last 30 seconds, emitted every 100ms to the frontend
    [Metric("Processing Time", "ms", Group = "vision", Aggregate = MetricAggregate.Average, EmitEveryMs = 100,
        MaxAgeSeconds = 30)]
    private PerformanceMetric<double> _processingTime;

    public override Task Init(CancellationToken token)
    {
        AddFilters();

        SubscribeImage(
            "front_view",
            OnImageReceived,
            token
        );

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
        // Get rid of super fine grained details to assist our hsv filter
        _filters.Add(new BlurFilter(5, 3, BlurFilter.BlurMethod.BoxBlur));

        // Ground hsv filter
        _filters.Add(new HsvFilter(Constants.VisionSubsystem.GroundThreshold));

        // Region of disinterest (defaults to remove within the region)
        _filters.Add(new RegionFilter(
            [new Point(0, 0), new Point(100, 100), new Point(50, 50)]
        ));

        // Top down transformation
        _filters.Add(new TopDownFilter(
            [
                new Point2f(220, 200), 
                new Point2f(420, 200), 
                new Point2f(580, 420), 
                new Point2f(60, 420)
            ],
            new Size(80, 80)
        ));

        // Inflation
        _filters.Add(new InflationFilter());
    }

    private async Task ImageProcessingTask(CancellationToken token)
    {
        var watch = new Stopwatch();
        try
        {
            while (!token.IsCancellationRequested)
            {
                var frame = await _frameChannel.Reader.ReadAsync(token);
                watch.Start();

                var mat = CvUtils.AsMat(frame);
                mat = _filters.Aggregate(mat, (current, filter) => filter.Apply(current));

                var frameBytes = CvUtils.FromMat(mat);
                var newFrame = MessageConstructor.CreateImageFrame(
                    frame.Width,
                    frame.Height,
                    "hsv_view",
                    frameBytes
                );

                var wrappedFrame = MessageWrapper.From(
                    MessageType.ImageFrame,
                    newFrame.ByteBuffer.ToFullArray()
                );

                EventBus.Instance.Publish(new MessageWrapperEvent(wrappedFrame));

                watch.Stop();
                _processingTime.AddSample(watch.ElapsedMilliseconds);
                watch.Reset();

                mat.Dispose();
            }
        }
        catch (OperationCanceledException)
        {
            // Expected on shutdown
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Vision processing task crashed");
        }
    }

    private Task OnImageReceived(ImageFrame frame, CancellationToken token)
    {
        _frameChannel.Writer.TryWrite(frame);
        return Task.CompletedTask;
    }
}