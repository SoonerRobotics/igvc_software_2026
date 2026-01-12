using System.Diagnostics;
using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Core.Performance;
using igvc_csharp.Events;
using igvc_csharp.MessageUtils;
using igvc_csharp.Subsystems.Vision.Filters;
using igvc_csharp.Utilities;
using Messages;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Vision;

[Subsystem("VisionSubsystem", Disabled = false)]
public class VisionSubsystem : SubsystemBase
{
    private readonly List<IFilter> _filters = [];
    private Task? _processTask;

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

        _processTask = Task.Factory.StartNew(
            () => ImageProcessingTask(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        return Task.CompletedTask;
    }

    private void AddFilters()
    {
        // Get rid of super fine grained details to assist our hsv filter
        _filters.Add(new BlurFilter(5, 3, BlurFilter.BlurMethod.BoxBlur));

        // Ground hsv filter
        _filters.Add(new HsvFilter(Constants.VisionSubsystem.GroundThreshold));
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

                using var mat = CvUtils.AsMat(frame);
                foreach (var filter in _filters)
                {
                    filter.Apply(mat);
                }

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

    public override Task Shutdown()
    {
        return Task.CompletedTask;
    }
}