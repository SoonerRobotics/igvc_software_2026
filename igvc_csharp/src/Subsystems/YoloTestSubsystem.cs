using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Core.Hardware;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using igvc_csharp.Yolo;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

[Subsystem("YoloTestSubsystem", Disabled = false)]
public class YoloTestSubsystem(
    ZedSubsystem zed
) : SubsystemBase
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

        SubscribeImage("zed", OnImageReceived, token);
        SubscribeMessage<ZedFrame>(MessageType.ZedFrame, OnZedFrameReceived, token);

        _ = Task.Run(() => InferenceLoop(token), token);

        SetOperatingState(SubsystemState.Operating);
        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        _detector?.Dispose();
        lock (_depthLock)
        {
            _latestDepthMat?.Dispose();
        }

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
            using var brightened = new Mat();
            src.ConvertTo(brightened, MatType.CV_8UC3, alpha: 1.5, beta: 0);

            var matJpeg = CvUtils.FromMat(brightened);
            var detections = _detector!.Detect(matJpeg);

            // Scale from YOLO output space → ZED frame space (1280×720)
            float scaleX = (float)ZedFrameSharedMemoryReader.FrameWidth / brightened.Width;
            float scaleY = (float)ZedFrameSharedMemoryReader.FrameHeight / brightened.Height;

            // Query depth for each detection concurrently — semaphore inside
            // RequestDepthAsync serializes them, but we don't need to do so ourselves
            var depthTasks = detections.Select(det =>
            {
                int cx = (int)((det.Bounding.X + det.Bounding.Width / 2f) * scaleX);
                int cy = (int)((det.Bounding.Y + det.Bounding.Height / 2f) * scaleY);
                return zed.RequestDepthAsync(cx, cy, timeoutMs: 200);
            }).ToList();

            var depthResults = await Task.WhenAll(depthTasks);

            var withDepth = detections.Select((det, i) =>
            {
                var response = depthResults[i];
                float distance = response is not null
                    ? MathF.Sqrt(response.Value.X * response.Value.X + response.Value.Y * response.Value.Y + response.Value.Z * response.Value.Z)
                    : float.NaN;
                return new DetectionWithDepth(det, distance);
            }).ToList();

            var annotated = OpenCvDetectionRenderer.RenderDetections(matJpeg, withDepth);
            var imageFrame = MessageConstructor.CreateImageFrame(640, 480, "yolo", annotated);
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