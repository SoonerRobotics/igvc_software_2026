using System.Runtime.InteropServices;
using SocketCANSharp;

namespace igvc_csharp.Subsystems.Hardware.CanLayers;

public class MotorControlLayer(CanbusSubsystem canbus)
{
    [StructLayout(LayoutKind.Explicit, Pack = 1)]
    private struct MotorControlPacket(short forwardVelocity, short sidewaysVelocity, short angularVelocity)
    {
        [FieldOffset(0)] public short ForwardVelocity = forwardVelocity;
        [FieldOffset(1)] public short SidewaysVelocity = sidewaysVelocity;
        [FieldOffset(2)] public short AngularVelocity = angularVelocity;
    }

    [StructLayout(LayoutKind.Explicit, Pack = 1)]
    public struct MotorOdometryPacket(short deltaX, short deltaY, short deltaTheta)
    {
        [FieldOffset(0)] public float DeltaX = deltaX * 0.0001f;
        [FieldOffset(1)] public float DeltaY = deltaY * 0.0001f;
        [FieldOffset(2)] public float DeltaTheta = deltaTheta * 0.001f;
    }

    public void SendCommand(float forwardVelocity, float sidewaysVelocity, float angularVelocity)
    {
        var fv = (byte)(forwardVelocity / 0.0001f);
        var sv = (byte)(sidewaysVelocity / 0.0001f);
        var av = (byte)(angularVelocity / 0.001f);
        var packet = new MotorControlPacket(fv, sv, av);
        var bytes = CanbusSubsystem.PacketToBytes(packet);
        canbus.SendCanFrame(new CanFrame(
            (uint)CanId.MotorCommand,
            bytes
        ));
    }

    public static MotorOdometryPacket? ParseFeedback(CanFrame frame)
    {
        if (frame.CanId != (uint)CanId.MotorOdometry)
        {
            return null;
        }

        var packet = CanbusSubsystem.PacketFromBytes<MotorOdometryPacket>(frame.Data);
        return packet;
    }
}