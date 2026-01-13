using SocketCANSharp;

namespace igvc_csharp.CanSpec;

using System;

public sealed class MotorCommandMessage(short forward, short sideways, short angular) : ICanMessage<MotorCommandMessage>
{
    public short ForwardVelocity { get; } = forward;
    public short SidewaysVelocity { get; } = sideways;
    public short AngularVelocity { get; } = angular;

    public static MotorCommandMessage Read(byte[] data)
    {
        if (data == null || data.Length < 6)
        {
            throw new ArgumentException("MotorCommandMessage requires exactly 6 bytes.");
        }

        return new MotorCommandMessage(
            BitConverter.ToInt16(data, 0),
            BitConverter.ToInt16(data, 2),
            BitConverter.ToInt16(data, 4)
        );
    }

    public CanFrame Write()
    {
        var data = new byte[6];

        BitConverter.GetBytes(ForwardVelocity).CopyTo(data, 0);
        BitConverter.GetBytes(SidewaysVelocity).CopyTo(data, 2);
        BitConverter.GetBytes(AngularVelocity).CopyTo(data, 4);

        return new CanFrame(
            (uint)CanId.MotorCommand,
            data
        );
    }
}