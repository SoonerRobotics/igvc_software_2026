using SocketCANSharp;

namespace igvc_csharp.Core.Hardware.Canbus;

public interface ICanInterface : IDisposable
{
    /// <summary>Whether the interface is currently open and ready.</summary>
    bool IsOpen { get; }

    /// <summary>
    /// Attempt to open the interface. Returns true on success, false if the
    /// underlying device is not available (e.g. interface not yet brought up).
    /// </summary>
    bool TryOpen();

    /// <summary>Close the interface and release resources.</summary>
    void Close();

    /// <summary>Write a CAN frame. Throws SocketCanException on hardware error.</summary>
    void Write(CanFrame frame);

    /// <summary>
    /// Blocking read. Returns the number of bytes read.
    /// Throws SocketCanException on hardware error, ObjectDisposedException if closed.
    /// </summary>
    int Read(out CanFrame frame);
}
