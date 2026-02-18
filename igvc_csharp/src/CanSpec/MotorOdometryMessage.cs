using SocketCANSharp;

namespace igvc_csharp.CanSpec;

using System;

public sealed class MotorOdometryMessage(double forward, double sideways, double angular) : ICanMessage<MotorOdometryMessage>
{
    public double DeltaX { get; } = forward;
    public double DeltaY { get; } = sideways;
    public double DeltaTheta { get; } = angular;

    public static MotorOdometryMessage Read(byte[] data)
    {
        if (data == null || data.Length < 6)
        {
            throw new ArgumentException("MotorOdometryMessage requires exactly 6 bytes.");
        }

        return new MotorOdometryMessage(
            BitConverter.ToInt16(data, 0) * 0.0001d,
            BitConverter.ToInt16(data, 2) * 0.0001d,
            BitConverter.ToInt16(data, 4) * 0.0001d
        );
    }

    public CanFrame Write()
    {
        // This should never be called
        return new CanFrame(
            (uint)CanId.MotorOdometry, []
        );
    }
}