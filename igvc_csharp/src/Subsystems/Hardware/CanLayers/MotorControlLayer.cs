using System.Runtime.InteropServices;
using SocketCANSharp;

namespace igvc_csharp.Subsystems.Hardware.CanLayers;

public class MotorControlLayer(CanbusSubsystem canbus)
{
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    private struct MotorControlPacket(short forwardVelocity, short sidewaysVelocity, short angularVelocity)
    {
        public short ForwardVelocity = forwardVelocity;
        public short SidewaysVelocity = sidewaysVelocity;
        public short AngularVelocity = angularVelocity;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct MotorOdometryPacket
    {
        public short RawDeltaX;
        public short RawDeltaY;
        public short RawDeltaTheta;

        public float DeltaX => RawDeltaX * 0.001f;
        public float DeltaY => RawDeltaY * 0.001f;
        public float DeltaTheta => RawDeltaTheta * 0.001f;
    }

    public void SetVelocities(double forwardVelocity, double sidewaysVelocity, double angularVelocity)
    {
        if (forwardVelocity > 0)
        {
            angularVelocity -= 0.04;
        }

        var fv = (short)(forwardVelocity / 0.001);
        var sv = (short)(sidewaysVelocity / 0.001);
        var av = (short)(angularVelocity / 0.001);

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
            return null;

        return CanbusSubsystem.PacketFromBytes<MotorOdometryPacket>(frame.Data);
    }
}