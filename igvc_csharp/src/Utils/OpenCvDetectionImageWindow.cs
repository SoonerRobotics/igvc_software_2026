using System.Collections.Concurrent;
using igvc_csharp.Yolo;
using OpenCvSharp;

namespace igvc_csharp.Utils;

public record DetectionWithDepth(Detection Detection, float DistanceMeters);
public sealed class OpenCvDetectionImageWindow : IDisposable
{
    private readonly string _windowName;
    private readonly BlockingCollection<(byte[], IReadOnlyList<DetectionWithDepth>)> _queue = new(1);
    private readonly Thread _uiThread;
    private bool _disposed;

    public OpenCvDetectionImageWindow(string windowName)
    {
        _windowName = windowName;
        _uiThread   = new Thread(UiLoop)
        {
            IsBackground = true,
            Name         = $"OpenCV:{windowName}"
        };
        _uiThread.Start();
    }

    public void EnqueueJpeg(byte[] jpeg, IReadOnlyList<DetectionWithDepth> detections)
    {
        if (_disposed) return;
        while (_queue.TryTake(out _)) { }
        _queue.Add((jpeg, detections));
    }

    private void UiLoop()
    {
        Cv2.NamedWindow(_windowName, WindowFlags.AutoSize);
        try
        {
            foreach (var item in _queue.GetConsumingEnumerable())
            {
                if (_disposed) break;

                var (jpeg, detections) = item;

                using var mat = Cv2.ImDecode(jpeg, ImreadModes.Color);
                if (mat.Empty()) continue;

                foreach (var d in detections)
                {
                    var color = DepthToColor(d.DistanceMeters);
                    Cv2.Rectangle(mat, d.Detection.Bounding, color, 2);

                    string label = float.IsNaN(d.DistanceMeters)
                        ? $"{d.Detection.Label} {d.Detection.Confidence:P1} (no depth)"
                        : $"{d.Detection.Label} {d.Detection.Confidence:P1} {d.DistanceMeters:F2}m";

                    var textOrg  = new Point(d.Detection.Bounding.X, d.Detection.Bounding.Y - 5);
                    var textSize = Cv2.GetTextSize(label, HersheyFonts.HersheySimplex, 0.5, 1, out int baseline);

                    Cv2.Rectangle(mat,
                        new Rect(textOrg.X, textOrg.Y - textSize.Height - baseline,
                            textSize.Width, textSize.Height + baseline),
                        Scalar.Black, -1);

                    Cv2.PutText(mat, label, textOrg,
                        HersheyFonts.HersheySimplex, 0.5, Scalar.White, 1);
                }

                Cv2.ImShow(_windowName, mat);
                Cv2.WaitKey(1);
            }
        }
        catch (ObjectDisposedException) { }
        catch (OperationCanceledException) { }
        finally
        {
            Cv2.DestroyWindow(_windowName);
        }
    }

    // Green = close, Red = far
    private static Scalar DepthToColor(float meters)
    {
        if (float.IsNaN(meters)) return Scalar.Gray;
        float t   = Math.Clamp((meters - 0.2f) / (10f - 0.2f), 0f, 1f);
        int   r   = (int)(255 * t);
        int   g   = (int)(255 * (1f - t));
        return new Scalar(0, g, r);
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        _queue.CompleteAdding();
        _uiThread.Join(2000);
        _queue.Dispose();
    }
}