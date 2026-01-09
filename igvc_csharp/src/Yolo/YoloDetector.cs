using Compunet.YoloSharp;

namespace igvc_csharp.Yolo;

using OpenCvSharp;

public sealed class YoloDetector : IDisposable
{
    private readonly YoloPredictor _predictor;

    public YoloDetector(string modelPath)
    {
        _predictor = new YoloPredictor(modelPath);
    }

    public IReadOnlyList<Detection> Detect(byte[] image)
    {
        var results = _predictor.Detect(image);

        var detections = new List<Detection>(results.Count);
        detections.AddRange(results.Select(r => new Detection(r.Name.Name, r.Confidence, new Rect((int)r.Bounds.X, (int)r.Bounds.Y, (int)r.Bounds.Width, (int)r.Bounds.Height))));

        return detections;
    }

    public void Dispose()
    {
        _predictor.Dispose();
    }
}