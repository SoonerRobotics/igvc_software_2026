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

    public IReadOnlyList<Detection> Detect(Mat image)
    {
        var results = _predictor.Predict(image);

        var detections = new List<Detection>(results.Count);

        foreach (var r in results)
        {
            detections.Add(new Detection(
                r.Label.Name,
                r.Confidence,
                new Rect(
                    (int)r.BoundingBox.X,
                    (int)r.BoundingBox.Y,
                    (int)r.BoundingBox.Width,
                    (int)r.BoundingBox.Height)
            ));
        }

        return detections;
    }

    public void Dispose()
    {
        _predictor.Dispose();
    }
}