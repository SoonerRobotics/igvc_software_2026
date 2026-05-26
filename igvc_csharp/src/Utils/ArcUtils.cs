using Google.FlatBuffers;
using igvc_csharp.Utils.Messages;
using Messages.Arc;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

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
    
    public static MessageWrapper CreateArcData_Json(string identifier, object obj)
    {
        var json = JsonConvert.SerializeObject(obj);
        var data = System.Text.Encoding.UTF8.GetBytes(json);
        return CreateArcDataWrapper(identifier, data);
    }
    
    public static T? ParseArcDataJson<T>(byte[] data)
    {
        var json = System.Text.Encoding.UTF8.GetString(data);
        return JsonConvert.DeserializeObject<T>(json);
    }
    
    public static JObject ParseArcDataJson(byte[] data)
    {
        var json = System.Text.Encoding.UTF8.GetString(data);
        return JObject.Parse(json);
    }
}