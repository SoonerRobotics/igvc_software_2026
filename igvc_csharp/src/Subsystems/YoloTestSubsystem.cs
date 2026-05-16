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

[Subsystem("YoloTestSubsystem", Disabled = false, DependsOn = [typeof(ArcSubsystem)])]
public class YoloTestSubsystem : SubsystemBase
{
    private YoloDetector? _detector;
    private readonly Channel<byte[]> _frameChannel = Channel.CreateBounded<byte[]>(1);

    private Mat? _latestDepthMat;
    private readonly object _depthLock = new();

    public override Task Init(CancellationToken token)
    {
        _detector = new YoloDetector(
            FileUtils.GetFileRelativeToRoot("resources/yolov8s-worldv2.onnx"),
            YoloExecutionProvider.Cuda
        );

        SubscribeImage("zed2i", OnImageReceived, token);
        SubscribeMessage<ZedFrame>(MessageType.ZedFrame, OnZedFrameReceived, token);

        _ = Task.Run(() => InferenceLoop(token), token);

        SetOperatingState(SubsystemState.Operating);
        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        _detector?.Dispose();
        lock (_depthLock) { _latestDepthMat?.Dispose(); }
        return Task.CompletedTask;
    }

    private Task OnZedFrameReceived(ZedFrame frame, CancellationToken token)
    {
        var depthMat = CvUtils.AsDepthMat(frame); // CV_32FC1, meters
        lock (_depthLock)
        {
            _latestDepthMat?.Dispose();
            _latestDepthMat = depthMat;
        }
        return Task.CompletedTask;
    }

    private async Task InferenceLoop(CancellationToken token)
    {
        await foreach (var jpeg in _frameChannel.Reader.ReadAllAsync(token))
        {
            using var src = CvUtils.AsMat(jpeg);

            // ConvertTo in-place is unreliable — always use a separate destination mat
            using var brightened = new Mat();
            src.ConvertTo(brightened, MatType.CV_8UC3, alpha: 1.5, beta: 0);

            // Flip vertically — ZED image arrives upside down from GPU readback
            using var flipped = new Mat();
            Cv2.Flip(brightened, flipped, FlipMode.X);

            var matJpeg    = CvUtils.FromMat(flipped);
            var detections = _detector!.Detect(matJpeg);

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

            var annotated    = OpenCvDetectionRenderer.RenderDetections(matJpeg, withDepth);
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