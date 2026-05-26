using System.Runtime.InteropServices;
using igvc_csharp.Utils;
using SocketCANSharp;

namespace igvc_csharp.Subsystems.Hardware.CanLayers;

public class SafetyLightsLayer(CanbusSubsystem canbus)
{
    private SafetyLightsPacket? _lastPacket;
    private CancellationTokenSource? _flashCts;
    private readonly object _flashLock = new();

    public enum SafetyLightsMode : byte
    {
        /// <summary>A default "booting" mode, whatever the firmware decides</summary>
        Default,
        /// <summary>A "chasing" or loading animation</summary>
        Chasing,
        /// <summary>The led strip is solid the current color</summary>
        Solid,
        /// <summary>The led strip is blinking the current color</summary>
        Blinking,
        /// <summary>RGB go brr but at full capacity</summary>
        Rainbow
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    private struct SafetyLightsPacket(SafetyLightsMode mode, ColorUtils.Color color, ushort speed)
    {
        public SafetyLightsMode Mode = mode;
        public byte R = color.R;
        public byte G = color.G;
        public byte B = color.B;
        public ushort Speed = speed;
        public ushort Unused = 0;
    }

    private void SetRaw(SafetyLightsPacket packet)
    {
        _lastPacket = packet;
        var bytes = CanbusSubsystem.PacketToBytes(packet);
        canbus.SendCanFrame(new CanFrame((uint)CanId.SafetyLights, bytes));
    }

    public void Set(SafetyLightsMode mode, ColorUtils.Color color, ushort speed = 2000)
        => SetRaw(new SafetyLightsPacket(mode, color, speed));

    public void SetAutonomous() => Set(SafetyLightsMode.Blinking, ColorUtils.Color.Autonomous);
    public void SetManual() => Set(SafetyLightsMode.Solid, ColorUtils.Color.Manual);
    public void SetBooting() => Set(SafetyLightsMode.Chasing, ColorUtils.Color.Amaranth, 1500);
    public void SetDisabled() => Set(SafetyLightsMode.Default, ColorUtils.Color.White);

    public void FlashTemporary(ColorUtils.Color color, CancellationToken token,
        ushort length = 2000, ushort speed = 1000)
    {
        SafetyLightsPacket restoreTo;
        CancellationTokenSource cts;

        lock (_flashLock)
        {
            _flashCts?.Cancel();
            _flashCts?.Dispose();

            restoreTo = _lastPacket
                ?? new SafetyLightsPacket(SafetyLightsMode.Default, ColorUtils.Color.White, 1000);

            cts = CancellationTokenSource.CreateLinkedTokenSource(token);
            _flashCts = cts;
        }

        Set(SafetyLightsMode.Blinking, color, speed);

        _ = Task.Run(async () =>
        {
            try
            {
                await Task.Delay(length, cts.Token);
                SetRaw(restoreTo);
            }
            catch (OperationCanceledException) { /* superseded or shutdown */ }
            finally
            {
                lock (_flashLock)
                {
                    if (_flashCts == cts)
                        _flashCts = null;
                }
                cts.Dispose();
            }
        }, CancellationToken.None);
    }
}