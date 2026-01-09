using System;
using OpenCvSharp;

namespace igvc_csharp.Utilities;

/// <summary>
/// Displays JPEG frames in an OpenCV window.
/// </summary>
public sealed class OpenCvImageWindow : IDisposable
{
    private readonly string _windowName;
    private bool _disposed;

    public OpenCvImageWindow(string windowName = "IGVC Vision")
    {
        _windowName = windowName;

        Cv2.NamedWindow(
            _windowName,
            WindowFlags.AutoSize | WindowFlags.GuiExpanded);
    }

    /// <summary>
    /// Decode JPEG bytes and display them.
    /// </summary>
    public void ShowJpeg(ReadOnlySpan<byte> jpegBytes)
    {
        if (_disposed)
            throw new ObjectDisposedException(nameof(OpenCvImageWindow));

        // Decode JPEG -> Mat
        using var mat = Cv2.ImDecode(
            jpegBytes.ToArray(),
            ImreadModes.Color);

        if (mat.Empty())
            return;

        Cv2.ImShow(_windowName, mat);

        // Required for OpenCV window to update
        Cv2.WaitKey(1);
    }

    /// <summary>
    /// Close the window and release resources.
    /// </summary>
    public void Dispose()
    {
        if (_disposed)
            return;

        _disposed = true;

        try
        {
            Cv2.DestroyWindow(_windowName);
        }
        catch
        {
            // ignore shutdown errors
        }
    }
}