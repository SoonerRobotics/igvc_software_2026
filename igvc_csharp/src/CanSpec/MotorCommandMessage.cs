using SocketCANSharp;

namespace igvc_csharp.CanSpec;

using System;

public sealed class MotorCommandMessage(double forward, double sideways, double angular) : ICanMessage<MotorCommandMessage>
{
    public double ForwardVelocity { get; set; } = forward;
    public double SidewaysVelocity { get; set; } = sideways;
    public double AngularVelocity { get; set; } = angular;

    public static MotorCommandMessage Read(byte[] data)
    {
        if (data == null || data.Length < 6)
        {
            throw new ArgumentException("MotorCommandMessage requires exactly 6 bytes.");
        }

        return new MotorCommandMessage(
            BitConverter.ToInt16(data, 0) * 0.0001f,
            BitConverter.ToInt16(data, 2) * 0.0001f,
            BitConverter.ToInt16(data, 4) * 0.0001f
        );
    }

    public CanFrame Write()
    {
        var data = new byte[6];

        BitConverter.GetBytes((short)(ForwardVelocity / 0.0001f)).CopyTo(data, 0);
        BitConverter.GetBytes((short)(SidewaysVelocity / 0.0001f)).CopyTo(data, 2);
        BitConverter.GetBytes((short)(AngularVelocity / 0.001f)).CopyTo(data, 4);

        return new CanFrame(
            (uint)CanId.MotorCommand,
            data
        );
    }
}