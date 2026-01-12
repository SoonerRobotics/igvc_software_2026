using System.Diagnostics;
using System.Threading.Channels;
using igvc_csharp;
using igvc_csharp.Core;
using igvc_csharp.Core.Performance;
using igvc_csharp.Events;
using igvc_csharp.MessageUtils;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Utilities;
using igvc_csharp.Yolo;
using Messages;
using Microsoft.Extensions.Logging;

// Just a testing subsystem for yolo

[Subsystem("YoloTestSubsystem", Disabled = false, DependsOn = [typeof(ArcSubsystem)])]
public class YoloTestSubsystem : SubsystemBase
{
    private YoloDetector? _detector;
    private readonly Channel<byte[]> _frameChannel = Channel.CreateBounded<byte[]>(1);
    private readonly Stopwatch _watch = new Stopwatch();

    // Metrics
    // Calculates the average detection time over the last 30 seconds, emitted every 100ms to the frontend
    [Metric("Detection Time", "ms", Group = "yolo", Aggregate = MetricAggregate.Average, EmitEveryMs = 100,
        MaxAgeSeconds = 30)]
    private PerformanceMetric<double> _detectionTime;

    public override Task Init(CancellationToken token)
    {
        _detector = new YoloDetector(FileUtiltiies.GetFileRelativeToRoot("resources/yolo11n.onnx"));

        SubscribeImage(
            "front_view",
            OnImageReceived,
            token
        );

        _ = Task.Run(() => InteferenceLoop(token), token);

        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        _detector?.Dispose();
        return Task.CompletedTask;
    }

    private async Task InteferenceLoop(CancellationToken token)
    {
        var reader = _frameChannel.Reader;

        await foreach (var jpeg in reader.ReadAllAsync(token))
        {
            _watch.Restart();
            var detections = _detector!.Detect(jpeg);
            _watch.Stop();
            _detectionTime.AddSample(_watch.ElapsedMilliseconds);
            var annotated = OpenCvDetectionRenderer.RenderDetections(jpeg, detections);
            var frame = MessageConstructor.CreateImageFrame(640, 480, "yolo_view", annotated);
            var wrappedFrame = MessageWrapper.From(MessageType.ImageFrame, frame.ByteBuffer.ToFullArray());
            EventBus.Instance.Publish(new MessageWrapperEvent(wrappedFrame));
        }
    }

    private Task OnImageReceived(ImageFrame frame, CancellationToken token)
    {
        _frameChannel.Writer.TryWrite(frame.GetImageDataArray());
        return Task.CompletedTask;
    }
}