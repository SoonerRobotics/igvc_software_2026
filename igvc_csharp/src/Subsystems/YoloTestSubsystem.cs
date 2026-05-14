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
using OpenCvSharp;

// Just a testing subsystem for yolo

[Subsystem("YoloTestSubsystem", Disabled = true, DependsOn = [typeof(ArcSubsystem)])]
public class YoloTestSubsystem : SubsystemBase
{
    private YoloDetector? _detector;
    private OpenCvDetectionImageWindow? _window;
    private readonly Channel<byte[]> _frameChannel = Channel.CreateBounded<byte[]>(1);

    public override Task Init(CancellationToken token)
    {
        // _detector = new YoloDetector(FileUtils.GetFileRelativeToRoot("resources/yolo11n.onnx"));
        _detector = new YoloDetector(FileUtils.GetFileRelativeToRoot("resources/yolov8s-worldv2.onnx"));
        _window = new OpenCvDetectionImageWindow("IGVC 2026 | Yolo Test");
        
        SubscribeImage(
            "zed2i",
            OnImageReceived,
            token
        );

        SubscribeMessage<ZedFrame>(MessageType.ZedFrame, OnZedFrameReceived, token);

        _ = Task.Run(() => InteferenceLoop(token), token);

        SetOperatingState(SubsystemState.Operating);
        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        _detector?.Dispose();
        lock (_depthLock) { _latestDepthMat?.Dispose(); }
        return Task.CompletedTask;
    }
    
    private Mat?  _latestDepthMat;
    private readonly object _depthLock = new();
    private Task OnZedFrameReceived(ZedFrame frame, CancellationToken token)
    {
        // Keep raw float depth mat for distance sampling
        var depthMat = CvUtils.AsDepthMat(frame); // CV_32FC1, meters

        lock (_depthLock)
        {
            _latestDepthMat?.Dispose();
            _latestDepthMat = depthMat;
        }

        // Also show colorized depth in the window while inference runs
        // var colorized = CvUtils.AsColorizedMat(frame);
        // _window?.EnqueueJpeg(CvUtils.FromMat(colorized), []);
        // colorized.Dispose();

        // Forward RGB jpeg to inference if you have it, or skip
        return Task.CompletedTask;
    }

    private async Task InteferenceLoop(CancellationToken token)
    {
        var reader = _frameChannel.Reader;

        await foreach (var jpeg in reader.ReadAllAsync(token))
        {
            var detections = _detector!.Detect(jpeg);

            // Sample depth at each detection center
            List<DetectionWithDepth> withDepth;
            lock (_depthLock)
            {
                withDepth = detections.Select(det => new DetectionWithDepth(
                    det,
                    _latestDepthMat != null
                        ? CvUtils.SampleDepth(_latestDepthMat, det.Bounding)
                        : float.NaN
                )).ToList();
            }

            _window?.EnqueueJpeg(jpeg, withDepth);

            // Annotated publish — pass withDepth labels
            var annotated    = OpenCvDetectionRenderer.RenderDetections(jpeg, detections);
            var imageFrame   = MessageConstructor.CreateImageFrame(640, 480, "yolo_view", annotated);
            var wrappedFrame = MessageWrapper.From(MessageType.ImageFrame, imageFrame.ByteBuffer.ToFullArray());
            EventBus.Instance.Publish(new MessageWrapperEvent(wrappedFrame));
        }
    }

    private Task OnImageReceived(ImageFrame frame, CancellationToken token)
    {
        _frameChannel.Writer.TryWrite(frame.GetImageDataArray());
        return Task.CompletedTask;
    }
}