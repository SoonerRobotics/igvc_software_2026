using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Messages;
using igvc_csharp.MessageUtils;
using igvc_csharp.Subsystems;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Utilities;
using igvc_csharp.Yolo;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

// Just a testing subsystem for yolo

[Subsystem("TestSubsystem", Disabled = true, DependsOn = [typeof(SimulatorSubsystem)])]
public class TestSubsystem(SimulatorSubsystem arc) : SubsystemBase
{
    private OpenCvDetectionImageWindow? _window;
    private YoloDetector? _detector;
    
    private readonly Channel<byte[]> _frameChannel =
        Channel.CreateBounded<byte[]>(1); // latest-frame-wins
    
    public override Task Init(CancellationToken token)
    {
        _window = new OpenCvDetectionImageWindow("Simulator Camera");
        _detector = new YoloDetector(FileUtilities.GetFileRelativeToRoot("resources/yolo11n.onnx"));
        
        SubscribeMessage<ImageFrame>(
            MessageType.ImageFrame,
            OnImageReceived,
            token
        );

        _ = Task.Run(() => InteferenceLoop(token), token);
        
        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        _detector?.Dispose();
        _window?.Dispose();
        return Task.CompletedTask;
    }

    private async Task InteferenceLoop(CancellationToken token)
    {
        var reader = _frameChannel.Reader;
        
        await foreach (var jpeg in reader.ReadAllAsync(token))
        {
            var detections = _detector!.Detect(jpeg);

            _window!.EnqueueJpeg(jpeg, detections);
        }
    }

    private Task OnImageReceived(ImageFrame frame, CancellationToken token)
    {
        _frameChannel.Writer.TryWrite(frame.GetImageDataArray());
        return Task.CompletedTask;
    }
}