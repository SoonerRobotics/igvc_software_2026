using Compunet.YoloSharp;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Yolo;

public enum YoloExecutionProvider
{
    Cpu,
    Cuda,
}

public sealed class YoloDetector : IDisposable
{
    private readonly YoloPredictor _predictor;
    private static readonly ILogger Logger = Logging.From<YoloDetector>();

    public YoloDetector(string modelPath, YoloExecutionProvider provider = YoloExecutionProvider.Cuda)
    {
        if (provider == YoloExecutionProvider.Cuda)
        {
            try
            {
                _predictor = new YoloPredictor(modelPath, new YoloPredictorOptions { UseCuda = true });
                Logger.LogInformation("[Yolo] Loaded {Model} on CUDA", modelPath);
                return;
            }
            catch (Exception ex)
            {
                Logger.LogWarning("[Yolo] CUDA unavailable ({Reason}), falling back to CPU", ex.Message);
            }
        }

        _predictor = new YoloPredictor(modelPath, new YoloPredictorOptions { UseCuda = false });
        Logger.LogInformation("[Yolo] Loaded {Model} on CPU", modelPath);
    }

    public IReadOnlyList<Detection> Detect(byte[] image)
    {
        var results    = _predictor.Detect(image);
        var detections = new List<Detection>(results.Count);
        detections.AddRange(results.Select(r => new Detection(
            r.Name.Name,
            r.Confidence,
            new Rect((int)r.Bounds.X, (int)r.Bounds.Y, (int)r.Bounds.Width, (int)r.Bounds.Height)
        )));
        return detections;
    }

    public void Dispose() => _predictor.Dispose();
}