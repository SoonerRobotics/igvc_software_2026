using System.Runtime.InteropServices;
using igvc_csharp.Utils;
using SocketCANSharp;

namespace igvc_csharp.Subsystems.Hardware.CanLayers;

public class SafetyLightsLayer(CanbusSubsystem canbus)
{
    private SafetyLightsPacket? _lastPacket;

    public enum SafetyLightsMode : byte
    {
        /// <summary>
        /// A default "booting" mode, whatever the firmware decides
        /// </summary>
        Default,
        
        /// <summary>
        /// A "chasing" or loading animation
        /// </summary>
        Chasing,

        /// <summary>
        /// The led strip is solid the current color
        /// </summary>
        Solid,

        /// <summary>
        /// The led strip is blinking the current color
        /// </summary>
        Blinking,

        /// <summary>
        /// RGB go brr but at full capacity
        /// </summary>
        Rainbow
    }

    [StructLayout(LayoutKind.Explicit, Pack = 1)]
    private struct SafetyLightsPacket(SafetyLightsMode mode, ColorUtils.Color color, ushort speed)
    {
        [FieldOffset(0)] public SafetyLightsMode Mode = mode;

        [FieldOffset(1)] public byte R = color.R;

        [FieldOffset(2)] public byte G = color.G;

        [FieldOffset(3)] public byte B = color.B;

        [FieldOffset(4)] public ushort Speed = speed;
    }

    private void SetRaw(SafetyLightsPacket packet)
    {
        _lastPacket = packet;
        var bytes = CanbusSubsystem.PacketToBytes(packet);
        canbus.SendCanFrame(new CanFrame(
            (uint)CanId.SafetyLights,
            bytes
        ));
    }

    // Public API

    public void Set(SafetyLightsMode mode, ColorUtils.Color color, ushort speed = 1000)
    {
        SetRaw(new SafetyLightsPacket(mode, color, speed));
    }

    public void SetAutonomous()
    {
        Set(SafetyLightsMode.Blinking, ColorUtils.Color.Autonomous);
    }

    public void SetManual()
    {
        Set(SafetyLightsMode.Solid, ColorUtils.Color.Manual);
    }

    public void SetBooting()
    {
        Set(SafetyLightsMode.Chasing, ColorUtils.Color.Amaranth, 1500);
    }

    public void SetDisabled()
    {
        Set(SafetyLightsMode.Default, ColorUtils.Color.White);
    }

    public void FlashTemporary(ColorUtils.Color color, CancellationToken token, ushort length = 2000,
        ushort speed = 1000)
    {
        var lastPacket = _lastPacket ?? new SafetyLightsPacket(SafetyLightsMode.Rainbow, ColorUtils.Color.White, 1000);
        Set(SafetyLightsMode.Blinking, color, speed);

        // After "length", set back to what it was before
        _ = Task.Run(async () =>
        {
            await Task.Delay(length, token);
            SetRaw(lastPacket);
        }, token);
    }

    // Safety

    static SafetyLightsLayer()
    {
        // Small safety measure to ensure there aren't issues deeper
        System.Diagnostics.Debug.Assert(
            Marshal.SizeOf<SafetyLightsPacket>() == 6,
            "SafetyLightsPacket size mismatch"
        );
    }
}