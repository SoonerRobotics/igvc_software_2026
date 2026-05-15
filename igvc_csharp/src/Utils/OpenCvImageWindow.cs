using System.Collections.Concurrent;
using OpenCvSharp;
using igvc_csharp.Core;

namespace igvc_csharp.Utils;

public sealed class OpenCvImageWindow : IDisposable
{
    private readonly string _windowName;
    private readonly BlockingCollection<byte[]> _queue = new(1); // latest-frame-wins
    private readonly Thread _uiThread;
    private bool _disposed;

    public OpenCvImageWindow(string windowName)
    {
        _windowName = windowName;

        if (Configuration.UseSimulation) {

            _uiThread = new Thread(UiLoop)
            {
                IsBackground = true,
                Name = $"OpenCV:{windowName}"
            };

            _uiThread.Start();
        }
    }

    public void EnqueueJpeg(byte[] jpeg)
    {
        if (_disposed) return;

        // Drop old frame, keep newest
        while (_queue.TryTake(out _)) { }
        _queue.Add(jpeg);
    }

    private void UiLoop()
    {
        Cv2.NamedWindow(_windowName, WindowFlags.AutoSize);

        try
        {
            while (!_disposed)
            {
                if (_queue.TryTake(out var jpeg, Timeout.Infinite))
                {
                    using var mat = Cv2.ImDecode(jpeg, ImreadModes.Color);
                    if (!mat.Empty())
                    {
                        Cv2.ImShow(_windowName, mat);
                    }
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
