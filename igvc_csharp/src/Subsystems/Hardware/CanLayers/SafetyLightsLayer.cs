using System.Runtime.InteropServices;
using igvc_csharp.Utils;
using SocketCANSharp;

namespace igvc_csharp.Subsystems.Hardware.CanLayers;

public class SafetyLightsLayer(CanbusSubsystem canbus)
{
    public enum SafetyLightsMode : byte
    {
        Disabled = 0,

        Manual = 1,

        AutoEnabled = 2,

        AutoDisabled = 3,

        LowPower = 4,

        TurnLeft = 5,

        TurnRight = 6,

        WaypointReached = 7,

        WaypointFollow = 8,

        Booting = 9,

        Shutdown = 10,
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    private struct SafetyLightsPacket(SafetyLightsMode mode)
    {
        public SafetyLightsMode Mode = mode;

        private readonly byte _r1 = 0;
        private readonly byte _r2 = 0;
        private readonly byte _r3 = 0;
        private readonly byte _r4 = 0;
        private readonly byte _r5 = 0;
        private readonly byte _r6 = 0;
        private readonly byte _r7 = 0;
    }

    private SafetyLightsMode _currentMode = SafetyLightsMode.Booting;

    private readonly object _flashLock = new();
    private CancellationTokenSource? _flashCts;

    private void Send(SafetyLightsMode mode)
    {
        _currentMode = mode;
        var bytes = CanbusSubsystem.PacketToBytes(new SafetyLightsPacket(mode));
        canbus.SendCanFrame(new CanFrame((uint)CanId.SafetyLights, bytes));
    }

    public void SetDisabled() => Send(SafetyLightsMode.Disabled);
    public void SetManual() => Send(SafetyLightsMode.Manual);
    public void SetAutoEnabled() => Send(SafetyLightsMode.AutoEnabled);
    public void SetAutoDisabled() => Send(SafetyLightsMode.AutoDisabled);
    public void SetLowPower() => Send(SafetyLightsMode.LowPower);
    public void SetTurnLeft() => Send(SafetyLightsMode.TurnLeft);
    public void SetTurnRight() => Send(SafetyLightsMode.TurnRight);
    public void SetBooting() => Send(SafetyLightsMode.Booting);
    public void SetShutdown() => Send(SafetyLightsMode.Shutdown);

    public void FlashWaypointReached(CancellationToken token, int duration = 3000)
        => FlashTemporary(SafetyLightsMode.WaypointReached, token, duration);

    public void FlashWaypointFollow(CancellationToken token, int duration = 3000)
        => FlashTemporary(SafetyLightsMode.WaypointFollow, token, duration);

    private void FlashTemporary(SafetyLightsMode flashMode, CancellationToken token, int duration)
    {
        SafetyLightsMode restore;
        CancellationTokenSource cts;

        lock (_flashLock)
        {
            _flashCts?.Cancel();
            _flashCts?.Dispose();

            restore = _currentMode;
            cts = CancellationTokenSource.CreateLinkedTokenSource(token);
            _flashCts = cts;
        }

        Send(flashMode);

        _ = Task.Run(async () =>
        {
            try
            {
                await Task.Delay(duration, cts.Token);
                Send(restore);
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