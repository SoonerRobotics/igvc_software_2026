using System.Text.Json;
using Google.FlatBuffers;
using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Utils.Messages;
using Messages.Arc;

namespace igvc_csharp.Utils;

public static class ArcUtils
{
    private static MessageWrapper CreateArcDataWrapper(string identifier, byte[] data)
    {
        var builder = new FlatBufferBuilder(1024);
        var idOffset = builder.CreateString(identifier);
        var dataOffset = ArcData.CreateDataPayloadVector(builder, data);
        ArcData.StartArcData(builder);
        ArcData.AddDataIdentifier(builder, idOffset);
        ArcData.AddDataPayload(builder, dataOffset);
        var offset = ArcData.EndArcData(builder);
        builder.Finish(offset.Value);
        return MessageWrapper.From(
            MessageType.ArcData,
            builder.SizedByteArray()
        );
    }

    public static MessageWrapper CreateArcData_PropertyChanged(string subsystem, string property, string value)
    {
        using var ms = new MemoryStream();
        using var bw = new BinaryWriter(ms);
        bw.Write(subsystem);
        bw.Write(property);
        bw.Write(value);
        var data = ms.ToArray();
        return CreateArcDataWrapper("property_changed", data);
    }

    public static MessageWrapper CreateArcData_Log(
        string cat,
        byte level,
        int id,
        string name,
        string message
    )
    {
        // Convert the log data to bytes
        using var ms = new MemoryStream();
        using var bw = new BinaryWriter(ms);
        bw.Write(cat);
        bw.Write(level);
        bw.Write(id);
        bw.Write(name);
        bw.Write(message);
        var data = ms.ToArray();
        return CreateArcDataWrapper("log", data);
    }

    public static MessageWrapper CreateArcData_RobotState(RobotState state)
    {
        // 1 bit mobility, 1 byte mode, 1 byte mission
        using var ms = new MemoryStream();
        using var bw = new BinaryWriter(ms);
        bw.Write(state.MotionAllowed ? (byte)1 : (byte)0);
        bw.Write((byte)state.Mode);
        bw.Write((byte)state.Mission);
        var data = ms.ToArray();
        return CreateArcDataWrapper("robot_state", data);
    }

    public static bool ExtractArcData_Mobility(ByteBuffer buffer)
    {
        var bytes = buffer.ToSizedArray(); // only the live portion
        using var ms = new MemoryStream(bytes);
        using var br = new BinaryReader(ms);
        return br.ReadInt32() != 0;
    }

    public static RobotModeEnum ExtractArcData_Mode(ByteBuffer buffer)
    {
        var bytes = buffer.ToSizedArray();
        using var ms = new MemoryStream(bytes);
        using var br = new BinaryReader(ms);
        return br.ReadInt32() switch
        {
            0 => RobotModeEnum.Disabled,
            1 => RobotModeEnum.Manual,
            2 => RobotModeEnum.Autonomous,
            _ => RobotModeEnum.Disabled
        };
    }

    public static MissionEnum ExtractArcData_Mission(ByteBuffer buffer)
    {
        var bytes = buffer.ToSizedArray();
        using var ms = new MemoryStream(bytes);
        using var br = new BinaryReader(ms);
        return br.ReadInt32() switch
        {
            0 => MissionEnum.Autonav,
            1 => MissionEnum.Selfdrive,
            _ => MissionEnum.Autonav
        };
    }
}