namespace igvc_csharp.Core.Hardware;

using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

public sealed class XboxController : IAsyncDisposable
{
    private const int InputEventSize = 24;

    private readonly string _devicePath;
    private FileStream? _stream;
    private CancellationTokenSource? _internalCts;

    private readonly Dictionary<EvAbs, InputAbsInfo> _absInfo = new();
    private readonly Dictionary<XboxAxis, float> _axisState = new();
    private readonly HashSet<XboxButton> _pressed = new();

    private int? _rumbleEffectId;

    public event Action<XboxController>? OnConnected;
    public event Action<XboxController>? OnDisconnected;

    // High-level decoded events (button/axis/other)
    public event Action<XboxControllerEvent>? OnEvent;

    // Optional: raw passthrough
    public event Action<RawInputEvent>? OnRawEvent;

    public string DevicePath => _devicePath;
    public bool IsConnected => _stream != null;

    public XboxController(string devicePath)
    {
        _devicePath = devicePath ?? throw new ArgumentNullException(nameof(devicePath));
    }

    public static IReadOnlyList<string> GetControllers()
    {
        var result = new List<string>();

        foreach (var eventDevice in Directory.GetFiles("/dev/input", "event*"))
        {
            var eventName = Path.GetFileName(eventDevice);
            var sysNamePath = $"/sys/class/input/{eventName}/device/name";
            if (!File.Exists(sysNamePath)) continue;

            var name = File.ReadAllText(sysNamePath).Trim();
            if (name.Contains("Xbox", StringComparison.OrdinalIgnoreCase) ||
                name.Contains("Microsoft", StringComparison.OrdinalIgnoreCase) ||
                name.Contains("X-Box", StringComparison.OrdinalIgnoreCase))
            {
                result.Add(eventDevice);
            }
        }

        return result;
    }
    
    public async Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        if (_stream != null) throw new InvalidOperationException("Controller already connected.");
        if (!File.Exists(_devicePath)) throw new FileNotFoundException("Device not found.", _devicePath);

        _internalCts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);

        _stream = new FileStream(
            _devicePath,
            FileMode.Open,
            FileAccess.ReadWrite,
            FileShare.ReadWrite,
            bufferSize: 4096,
            FileOptions.Asynchronous | FileOptions.SequentialScan);

        PrimeAbsInfo();

        OnConnected?.Invoke(this);
        var buffer = new byte[InputEventSize];
        try
        {
            while (!_internalCts.Token.IsCancellationRequested)
            {
                await ReadExactAsync(_stream, buffer, _internalCts.Token);

                var raw = ParseRawInputEvent(buffer);
                OnRawEvent?.Invoke(raw);

                if (!TryTranslate(raw, out var decoded) || decoded is null)
                {
                    continue;
                }
                
                ApplyState(decoded);
                OnEvent?.Invoke(decoded);
            }
        }
        catch (OperationCanceledException)
        {
            // expected
        }
        catch
        {
            // treat as disconnect
        }
        finally
        {
            await DisconnectInternalAsync();
        }
    }

    private async Task DisconnectAsync()
    {
        await _internalCts?.CancelAsync()!;
        await DisconnectInternalAsync();
    }

    public async ValueTask DisposeAsync()
    {
        await DisconnectAsync();
    }

    private bool TryTranslate(RawInputEvent raw, out XboxControllerEvent? decoded)
    {
        decoded = null;

        if ((EvType)raw.Type == EvType.EV_KEY)
        {
            if (TryMapButton((EvKey)raw.Code, out var button))
            {
                var isDown = raw.Value != 0;
                decoded = new XboxButtonEvent(button, isDown);
                return true;
            }

            return false;
        }

        if ((EvType)raw.Type == EvType.EV_ABS)
        {
            var abs = (EvAbs)raw.Code;

            if (abs == EvAbs.ABS_HAT0X)
            {
                decoded = new XboxDpadEvent(raw.Value, GetHatY());
                return true;
            }

            if (abs == EvAbs.ABS_HAT0Y)
            {
                decoded = new XboxDpadEvent(GetHatX(), raw.Value);
                return true;
            }

            if (TryMapAxis(abs, out var axis))
            {
                var norm = NormalizeAxis(abs, raw.Value, axis);
                decoded = new XboxAxisEvent(axis, norm, raw.Value);
                return true;
            }

            return false;
        }

        return false;
    }

    private void ApplyState(XboxControllerEvent ev)
    {
        switch (ev)
        {
            case XboxButtonEvent b:
                if (b.IsDown) _pressed.Add(b.Button);
                else _pressed.Remove(b.Button);
                break;

            case XboxAxisEvent a:
                _axisState[a.Axis] = a.Value;
                break;

            case XboxDpadEvent d:
                _axisState[XboxAxis.DpadX] = ClampHat(d.X);
                _axisState[XboxAxis.DpadY] = ClampHat(d.Y);
                break;
        }
    }
    
    public async Task RumbleAsync(float strong, float weak, TimeSpan duration, CancellationToken ct = default)
    {
        if (_stream is null) throw new InvalidOperationException("Not connected.");
        if (duration <= TimeSpan.Zero) throw new ArgumentOutOfRangeException(nameof(duration));

        strong = Clamp01(strong);
        weak = Clamp01(weak);

        var effectId = UploadOrUpdateRumbleEffect(_stream.SafeFileHandle.DangerousGetHandle(), strong, weak, duration);
        await WriteForceFeedbackPlayAsync(_stream, effectId, play: true, ct);
        try
        {
            await Task.Delay(duration, ct);
        }
        finally
        {
            await WriteForceFeedbackPlayAsync(_stream, effectId, play: false, ct);
            EraseEffect(_stream.SafeFileHandle.DangerousGetHandle(), effectId);
            _rumbleEffectId = null;
        }
    }

    private int UploadOrUpdateRumbleEffect(nint fd, float strong, float weak, TimeSpan duration)
    {
        var effect = new FfEffect
        {
            type = (ushort)FfEffectType.FF_RUMBLE,
            id = (short)(_rumbleEffectId ?? -1),
            direction = 0,
            trigger = new FfTrigger { button = 0, interval = 0 },
            replay = new FfReplay
            {
                length = (ushort)Math.Clamp((int)duration.TotalMilliseconds, 1, ushort.MaxValue),
                delay = 0
            },
            u = new FfEffectUnion
            {
                rumble = new FfRumbleEffect
                {
                    strong_magnitude = (ushort)(strong * ushort.MaxValue),
                    weak_magnitude = (ushort)(weak * ushort.MaxValue)
                }
            }
        };

        Console.WriteLine($"FfEffect Size: {Marshal.SizeOf<FfEffect>()}");
        var req = Ioctl.EVIOCSFF;
        if (Ioctl.ioctl(fd, req, ref effect) < 0)
        {
            Console.WriteLine("Failed to write ioctl: " + Marshal.GetLastWin32Error());
            throw new IOException($"EVIOCSFF failed (errno={Marshal.GetLastWin32Error()}).");
        }

        Console.WriteLine($"Wrote Effect: id=${effect.id}");
        _rumbleEffectId = effect.id;
        return effect.id;
    }

    private static void EraseEffect(nint fd, int effectId)
    {
        var req = Ioctl.EVIOCRMFF;
        var id = effectId;
        if (Ioctl.ioctl(fd, req, ref id) < 0)
        {
        }
    }

    private static async Task WriteForceFeedbackPlayAsync(FileStream stream, int effectId, bool play, CancellationToken ct)
    {
        Span<byte> buf = stackalloc byte[InputEventSize];
        buf.Clear(); // timeval=0

        BinaryPrimitives.WriteUInt16LittleEndian(buf.Slice(16, 2), (ushort)EvType.EV_FF);
        BinaryPrimitives.WriteUInt16LittleEndian(buf.Slice(18, 2), (ushort)effectId);
        BinaryPrimitives.WriteInt32LittleEndian(buf.Slice(20, 4), play ? 1 : 0);

        await stream.WriteAsync(buf.ToArray(), ct);
        await stream.FlushAsync(ct);
    }

    private float NormalizeAxis(EvAbs abs, int rawValue, XboxAxis axis)
    {
        if (_absInfo.TryGetValue(abs, out var info))
        {
            var clamped = Math.Clamp(rawValue, info.minimum, info.maximum);
            if (IsCenteredAxis(axis))
            {
                var center = (info.minimum + info.maximum) / 2;
                if (info.flat > 0 && Math.Abs(clamped - center) <= info.flat)
                    return 0f;

                if (clamped >= center)
                    return (float)(clamped - center) / Math.Max(1, info.maximum - center);
                else
                    return (float)(clamped - center) / Math.Max(1, center - info.minimum);
            }
            else
            {
                var denom = Math.Max(1, info.maximum - info.minimum);
                return (float)(clamped - info.minimum) / denom;
            }
        }

        return axis switch
        {
            XboxAxis.LeftX or XboxAxis.LeftY or XboxAxis.RightX or XboxAxis.RightY
                => Math.Clamp(rawValue / 32767f, -1f, 1f),

            XboxAxis.LeftTrigger or XboxAxis.RightTrigger
                => Math.Clamp(rawValue / 1023f, 0f, 1f),

            XboxAxis.DpadX or XboxAxis.DpadY
                => Math.Clamp(rawValue, -1f, 1f),

            _ => 0f
        };
    }

    private static bool IsCenteredAxis(XboxAxis axis)
        => axis is XboxAxis.LeftX or XboxAxis.LeftY or XboxAxis.RightX or XboxAxis.RightY;

    private static float ClampHat(int v) => v switch { < 0 => -1f, > 0 => 1f, _ => 0f };
    private static float Clamp01(float v) => Math.Clamp(v, 0f, 1f);

    private int GetHatX()
        => _axisState.TryGetValue(XboxAxis.DpadX, out var v) ? (int)v : 0;

    private int GetHatY()
        => _axisState.TryGetValue(XboxAxis.DpadY, out var v) ? (int)v : 0;

    private void PrimeAbsInfo()
    {
        if (_stream is null) return;

        var fd = _stream.SafeFileHandle.DangerousGetHandle();

        var axes = new[]
        {
            EvAbs.ABS_X, EvAbs.ABS_Y, EvAbs.ABS_RX, EvAbs.ABS_RY,
            EvAbs.ABS_Z, EvAbs.ABS_RZ,
            EvAbs.ABS_HAT0X, EvAbs.ABS_HAT0Y
        };

        foreach (var abs in axes)
        {
            if (TryGetAbsInfo(fd, abs, out var info))
            {
                _absInfo[abs] = info;
            }
        }
    }

    private static bool TryGetAbsInfo(nint fd, EvAbs abs, out InputAbsInfo info)
    {
        info = default;
        var req = Ioctl.EVIOCGABS((int)abs);

        if (Ioctl.ioctl(fd, req, ref info) < 0)
            return false;

        return true;
    }

    private static async Task ReadExactAsync(FileStream stream, byte[] buffer, CancellationToken ct)
    {
        int read = 0;
        while (read < buffer.Length)
        {
            var n = await stream.ReadAsync(buffer.AsMemory(read, buffer.Length - read), ct);
            if (n == 0) throw new IOException("Device disconnected.");
            read += n;
        }
    }

    private static RawInputEvent ParseRawInputEvent(byte[] buffer)
    {
        var type = BinaryPrimitives.ReadUInt16LittleEndian(buffer.AsSpan(16, 2));
        var code = BinaryPrimitives.ReadUInt16LittleEndian(buffer.AsSpan(18, 2));
        var value = BinaryPrimitives.ReadInt32LittleEndian(buffer.AsSpan(20, 4));
        return new RawInputEvent(type, code, value);
    }

    private async Task DisconnectInternalAsync()
    {
        if (_stream != null && _rumbleEffectId is int id)
        {
            try
            {
                await WriteForceFeedbackPlayAsync(_stream, id, play: false, CancellationToken.None);
                EraseEffect(_stream.SafeFileHandle.DangerousGetHandle(), id);
            }
            catch
            {
            }
            finally
            {
                _rumbleEffectId = null;
            }
        }

        if (_stream != null)
        {
            try
            {
                await _stream.DisposeAsync();
            }
            catch
            {
            }
            finally
            {
                _stream = null;
            }
        }

        _internalCts?.Dispose();
        _internalCts = null;

        OnDisconnected?.Invoke(this);
    }

    private static bool TryMapButton(EvKey key, out XboxButton button)
    {
        button = key switch
        {
            EvKey.BTN_SOUTH => XboxButton.A,
            EvKey.BTN_EAST => XboxButton.B,
            EvKey.BTN_NORTH => XboxButton.Y,
            EvKey.BTN_WEST => XboxButton.X,

            EvKey.BTN_TL => XboxButton.LeftBumper,
            EvKey.BTN_TR => XboxButton.RightBumper,
            EvKey.BTN_TL2 => XboxButton.LeftTriggerClick, // some devices expose as button too
            EvKey.BTN_TR2 => XboxButton.RightTriggerClick,

            EvKey.BTN_SELECT => XboxButton.View,
            EvKey.BTN_START => XboxButton.Menu,
            EvKey.BTN_MODE => XboxButton.Xbox,

            EvKey.BTN_THUMBL => XboxButton.LeftStick,
            EvKey.BTN_THUMBR => XboxButton.RightStick,

            EvKey.BTN_DPAD_UP => XboxButton.DpadUp,
            EvKey.BTN_DPAD_DOWN => XboxButton.DpadDown,
            EvKey.BTN_DPAD_LEFT => XboxButton.DpadLeft,
            EvKey.BTN_DPAD_RIGHT => XboxButton.DpadRight,

            _ => default
        };

        return button != default;
    }

    private static bool TryMapAxis(EvAbs abs, out XboxAxis axis)
    {
        axis = abs switch
        {
            EvAbs.ABS_X => XboxAxis.LeftX,
            EvAbs.ABS_Y => XboxAxis.LeftY,
            EvAbs.ABS_RX => XboxAxis.RightX,
            EvAbs.ABS_RY => XboxAxis.RightY,
            EvAbs.ABS_Z => XboxAxis.LeftTrigger,
            EvAbs.ABS_RZ => XboxAxis.RightTrigger,
            EvAbs.ABS_HAT0X => XboxAxis.DpadX,
            EvAbs.ABS_HAT0Y => XboxAxis.DpadY,
            _ => default
        };

        return axis != default;
    }
}

public abstract record XboxControllerEvent;
public sealed record XboxButtonEvent(XboxButton Button, bool IsDown) : XboxControllerEvent;
public sealed record XboxAxisEvent(XboxAxis Axis, float Value, int RawValue) : XboxControllerEvent;
public sealed record XboxDpadEvent(int X, int Y) : XboxControllerEvent;
public readonly record struct RawInputEvent(ushort Type, ushort Code, int Value);

public enum XboxButton
{
    None = 0,
    A,
    B,
    X,
    Y,
    LeftBumper,
    RightBumper,
    LeftStick,
    RightStick,
    View,
    Menu,
    Xbox,
    DpadUp,
    DpadDown,
    DpadLeft,
    DpadRight,
    LeftTriggerClick,
    RightTriggerClick
}

public enum XboxAxis
{
    None = 0,
    LeftX,
    LeftY,
    RightX,
    RightY,
    LeftTrigger,
    RightTrigger,
    DpadX,
    DpadY
}

public enum EvType : ushort
{
    EV_SYN = 0x00,
    EV_KEY = 0x01,
    EV_ABS = 0x03,
    EV_FF = 0x15
}

public enum EvAbs : ushort
{
    ABS_X = 0x00,
    ABS_Y = 0x01,
    ABS_Z = 0x02,
    ABS_RX = 0x03,
    ABS_RY = 0x04,
    ABS_RZ = 0x05,
    ABS_HAT0X = 0x10,
    ABS_HAT0Y = 0x11
}

public enum EvKey : ushort
{
    BTN_SOUTH = 0x130,
    BTN_EAST = 0x131,
    BTN_NORTH = 0x133,
    BTN_WEST = 0x134,

    BTN_TL = 0x136,
    BTN_TR = 0x137,
    BTN_TL2 = 0x138,
    BTN_TR2 = 0x139,

    BTN_SELECT = 0x13a,
    BTN_START = 0x13b,
    BTN_MODE = 0x13c,

    BTN_THUMBL = 0x13d,
    BTN_THUMBR = 0x13e,

    BTN_DPAD_UP = 0x220,
    BTN_DPAD_DOWN = 0x221,
    BTN_DPAD_LEFT = 0x222,
    BTN_DPAD_RIGHT = 0x223
}

internal static class Ioctl
{
    private const int IOC_NRBITS = 8;
    private const int IOC_TYPEBITS = 8;
    private const int IOC_SIZEBITS = 14;
    private const int IOC_DIRBITS = 2;

    private const int IOC_NRSHIFT = 0;
    private const int IOC_TYPESHIFT = IOC_NRSHIFT + IOC_NRBITS;
    private const int IOC_SIZESHIFT = IOC_TYPESHIFT + IOC_TYPEBITS;
    private const int IOC_DIRSHIFT = IOC_SIZESHIFT + IOC_SIZEBITS;

    private const int IOC_NONE = 0;
    private const int IOC_WRITE = 1;
    private const int IOC_READ = 2;

    private static uint IOC(int dir, int type, int nr, int size)
        => (uint)((dir << IOC_DIRSHIFT) |
                  (type << IOC_TYPESHIFT) |
                  (nr << IOC_NRSHIFT) |
                  (size << IOC_SIZESHIFT));

    private static uint IOR(int type, int nr, int size) => IOC(IOC_READ, type, nr, size);
    private static uint IOW(int type, int nr, int size) => IOC(IOC_WRITE, type, nr, size);

    public static uint EVIOCGABS(int absCode)
        => IOR('E', 0x40 + absCode, Marshal.SizeOf<InputAbsInfo>());

    public static readonly uint EVIOCSFF
        = IOW('E', 0x80, Marshal.SizeOf<FfEffect>());

    public static readonly uint EVIOCRMFF
        = IOW('E', 0x81, Marshal.SizeOf<int>());

    [DllImport("libc", SetLastError = true)]
    public static extern int ioctl(nint fd, uint request, ref InputAbsInfo data);

    [DllImport("libc", SetLastError = true)]
    public static extern int ioctl(nint fd, uint request, ref FfEffect data);

    [DllImport("libc", SetLastError = true)]
    public static extern int ioctl(nint fd, uint request, ref int data);
}

[StructLayout(LayoutKind.Sequential)]
internal struct InputAbsInfo
{
    public int value;
    public int minimum;
    public int maximum;
    public int fuzz;
    public int flat;
    public int resolution;
}

internal enum FfEffectType : ushort
{
    FF_RUMBLE = 0x50
}

[StructLayout(LayoutKind.Sequential)]
internal struct FfTrigger
{
    public ushort button;
    public ushort interval;
}

[StructLayout(LayoutKind.Sequential)]
internal struct FfReplay
{
    public ushort length;
    public ushort delay;
}

[StructLayout(LayoutKind.Sequential)]
internal struct FfRumbleEffect
{
    public ushort strong_magnitude;
    public ushort weak_magnitude;
}

[StructLayout(LayoutKind.Explicit, Size = 32)]
internal struct FfEffectUnion
{
    [FieldOffset(0)] public FfRumbleEffect rumble;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
internal struct FfEffect
{
    public ushort type;
    public short id;
    public ushort direction;
    public ushort _padding;
    public FfTrigger trigger;
    public FfReplay replay;
    public FfEffectUnion u;
}