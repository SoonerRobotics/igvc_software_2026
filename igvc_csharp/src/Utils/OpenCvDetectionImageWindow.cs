using System.Collections.Concurrent;
using igvc_csharp.Yolo;
using OpenCvSharp;

namespace igvc_csharp.Utils;

public sealed class OpenCvDetectionImageWindow : IDisposable
{
    private readonly string _windowName;
    private readonly BlockingCollection<(byte[], IReadOnlyList<Detection>)> _queue = new(1); // latest-frame-wins
    private readonly Thread _uiThread;
    private bool _disposed;

    public OpenCvDetectionImageWindow(string windowName)
    {
        _windowName = windowName;

        _uiThread = new Thread(UiLoop)
        {
            IsBackground = true,
            Name = $"OpenCV:{windowName}"
        };

        _uiThread.Start();
    }

    public void EnqueueJpeg(byte[] jpeg, IReadOnlyList<Detection> detection)
    {
        if (_disposed) return;

        // Drop old frame, keep newest
        while (_queue.TryTake(out _)) { }
        _queue.Add((jpeg, detection));
    }

    private void UiLoop()
    {
        Cv2.NamedWindow(_windowName, WindowFlags.AutoSize);

        try
        {
            while (!_disposed)
            {
                if (_queue.TryTake(out var item, Timeout.Infinite))
                {
                    var (jpeg, detections) = item;

                    using var mat = Cv2.ImDecode(jpeg, ImreadModes.Color);

                    foreach (var det in detections)
                    {
                        Cv2.Rectangle(mat, det.Bounding, Scalar.Red, 2);

                        Cv2.PutText(
                            mat,
                            $"{det.Label} {det.Confidence:P1}",
                            new Point(det.Bounding.X, det.Bounding.Y - 5),
                            HersheyFonts.HersheySimplex,
                            0.5,
                            Scalar.Yellow,
                            1);
                    }

                    Cv2.ImShow(_windowName, mat);
                }

                Cv2.WaitKey(1);
            }
        }
        finally
        {
            Cv2.DestroyWindow(_windowName);
        }
    }

    public void Dispose()
    {
        _disposed = true;
        _queue.CompleteAdding();
    }
}