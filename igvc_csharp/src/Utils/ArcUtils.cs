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
}