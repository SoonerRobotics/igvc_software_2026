using SocketCANSharp;
using SocketCANSharp.Network;

namespace igvc_csharp.Core.Hardware.Canbus;

/// <summary>
/// SocketCAN implementation - works for both slcan and candleLight (gs_usb)
/// devices, since both present as standard netdevs on Linux.
/// </summary>
public sealed class SocketCanInterface : ICanInterface
{
    private readonly string _interfaceName;
    private RawCanSocket? _socket;

    public bool IsOpen { get; private set; }

    public SocketCanInterface(string interfaceName)
    {
        _interfaceName = interfaceName;
    }

    public bool TryOpen()
    {
        if (IsOpen)
        {
            return true;
        }

        var network = CanNetworkInterface
            .GetAllInterfaces(true)
            .FirstOrDefault(i => i.Name.Equals(_interfaceName));

        if (network == null)
        {
            return false;
        }

        try
        {
            _socket = new RawCanSocket();
            _socket.Bind(network);
            IsOpen = true;
            return true;
        }
        catch
        {
            _socket?.Dispose();
            _socket = null;
            return false;
        }
    }

    public void Close()
    {
        IsOpen = false;
        _socket?.Dispose();
        _socket = null;
    }

    public void Write(CanFrame frame)
    {
        if (_socket == null)
        {
            throw new ObjectDisposedException(nameof(SocketCanInterface));
        }

        _socket.Write(frame);
    }

    public int Read(out CanFrame frame)
    {
        if (_socket == null)
        {
            throw new ObjectDisposedException(nameof(SocketCanInterface));
        }

        return _socket.Read(out frame);
    }

    public void Dispose() => Close();
}