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
                    DrawDetection(mat, d);

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

    private static void DrawDetection(Mat mat, DetectionWithDepth d)
    {
        var box   = d.Detection.Bounding;
        var color = DepthToColor(d.DistanceMeters);

        string distStr = float.IsNaN(d.DistanceMeters) ? "no depth" : $"{d.DistanceMeters:F2}m";
        string label   = $"{d.Detection.Label}  {distStr}";

        // --- Corner accents ---
        int cornerLen = Math.Min(box.Width, box.Height) / 5;
        DrawCornerAccents(mat, box, color, cornerLen, thickness: 2);

        // --- Leader line: top-center up, elbow, then horizontal ---
        var lineStart = new Point(box.X + box.Width / 2, box.Y);
        var lineElbow = new Point(box.X + box.Width / 2, box.Y - 20);
        var lineEnd   = new Point(lineElbow.X + 40,      lineElbow.Y);

        Cv2.Line(mat, lineStart, lineElbow, color, 1, LineTypes.AntiAlias);
        Cv2.Line(mat, lineElbow, lineEnd,   color, 1, LineTypes.AntiAlias);

        // --- Pill label ---
        const double scale     = 0.45;
        const int    textThick = 1;
        var textSize = Cv2.GetTextSize(label, HersheyFonts.HersheySimplex, scale, textThick, out _);

        const int padX = 6, padY = 4;
        var pillTl = new Point(lineEnd.X, lineEnd.Y - textSize.Height - padY);
        var pillBr = new Point(lineEnd.X + textSize.Width + padX * 2, lineEnd.Y + padY);

        Cv2.Rectangle(mat,
            new Rect(pillTl.X, pillTl.Y, pillBr.X - pillTl.X, pillBr.Y - pillTl.Y),
            color, thickness: -1);

        Cv2.PutText(mat, label,
            new Point(pillTl.X + padX, lineEnd.Y),
            HersheyFonts.HersheySimplex, scale,
            Scalar.White, textThick, LineTypes.AntiAlias);
    }

    private static void DrawCornerAccents(Mat mat, Rect box, Scalar color, int len, int thickness)
    {
        var tl = new Point(box.X,             box.Y);
        var tr = new Point(box.X + box.Width, box.Y);
        var bl = new Point(box.X,             box.Y + box.Height);
        var br = new Point(box.X + box.Width, box.Y + box.Height);

        // Top-left
        Cv2.Line(mat, tl, tl + new Point(len,  0),   color, thickness, LineTypes.AntiAlias);
        Cv2.Line(mat, tl, tl + new Point(0,    len),  color, thickness, LineTypes.AntiAlias);
        // Top-right
        Cv2.Line(mat, tr, tr + new Point(-len, 0),   color, thickness, LineTypes.AntiAlias);
        Cv2.Line(mat, tr, tr + new Point(0,    len),  color, thickness, LineTypes.AntiAlias);
        // Bottom-left
        Cv2.Line(mat, bl, bl + new Point(len,  0),   color, thickness, LineTypes.AntiAlias);
        Cv2.Line(mat, bl, bl + new Point(0,    -len), color, thickness, LineTypes.AntiAlias);
        // Bottom-right
        Cv2.Line(mat, br, br + new Point(-len, 0),   color, thickness, LineTypes.AntiAlias);
        Cv2.Line(mat, br, br + new Point(0,    -len), color, thickness, LineTypes.AntiAlias);
    }

    // Green (close) → Red (far), Gray if no depth
    private static Scalar DepthToColor(float meters)
    {
        if (float.IsNaN(meters)) return Scalar.Gray;
        float t = Math.Clamp((meters - 0.2f) / (10f - 0.2f), 0f, 1f);
        return new Scalar(0, (int)(200 * (1f - t)), (int)(200 * t));
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