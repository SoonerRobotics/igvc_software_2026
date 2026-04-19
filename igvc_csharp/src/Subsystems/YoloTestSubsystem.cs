using System.Diagnostics;
using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using igvc_csharp.Yolo;
using Messages;
using Microsoft.Extensions.Logging;

// Just a testing subsystem for yolo

[Subsystem("YoloTestSubsystem", Disabled = false, DependsOn = [typeof(ArcSubsystem)])]
public class YoloTestSubsystem : SubsystemBase
{
    private YoloDetector? _detector;
    private readonly Channel<byte[]> _frameChannel = Channel.CreateBounded<byte[]>(1);

    public override Task Init(CancellationToken token)
    {
        _detector = new YoloDetector(FileUtils.GetFileRelativeToRoot("resources/yolo11n.onnx"));

        SubscribeImage(
            "front_view",
            OnImageReceived,
            token
        );

        _ = Task.Run(() => InteferenceLoop(token), token);

        SetOperatingState(SubsystemState.Operating);
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
            var detections = _detector!.Detect(jpeg);
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