using igvc_csharp.Core.Config;
using igvc_csharp.Utils.Messages;
using System.Text.Json;
using Google.FlatBuffers;
using Messages.Arc;
using igvc_csharp.Utils;

namespace igvc_csharp.Subsystems.Arc.Config;

public static class ArcConfigIdentifiers
{
    public const string ConfigSnapshot = "config_snapshot";
    public const string ConfigKeyChanged = "config_key_changed";
    public const string ConfigPresetList = "config_preset_list";
    public const string ConfigAck = "config_ack";
}

public static class ArcConfigMessageFactory
{
    public static MessageWrapper CreateSnapshot(IReadOnlyDictionary<string, IConfigBinding> bindings)
    {
        var keys = new Dictionary<string, object?>();
        foreach (var (path, binding) in bindings)
            keys[path] = binding.Serialize();

        var payload = JsonSerializer.SerializeToUtf8Bytes(new
        {
            keys,
            presets = GetPresetNames(),
            currentPreset = BootParameters.Preset
        });

        return WrapArcData(ArcConfigIdentifiers.ConfigSnapshot, payload);
    }

    public static MessageWrapper CreateKeyChanged(string path, object? value)
    {
        var payload = JsonSerializer.SerializeToUtf8Bytes(new { path, value });
        return WrapArcData(ArcConfigIdentifiers.ConfigKeyChanged, payload);
    }

    public static MessageWrapper CreatePresetList()
    {
        var payload = JsonSerializer.SerializeToUtf8Bytes(new
        {
            presets = GetPresetNames(),
            currentPreset = BootParameters.Preset
        });
        return WrapArcData(ArcConfigIdentifiers.ConfigPresetList, payload);
    }

    public static MessageWrapper CreateAck(bool success, string? message = null)
    {
        var payload = JsonSerializer.SerializeToUtf8Bytes(new { success, message });
        return WrapArcData(ArcConfigIdentifiers.ConfigAck, payload);
    }

    private static string[] GetPresetNames()
    {
        var dir = FileUtils.ExpandPath(Configuration.Config.PresetsDirectory);
        if (!Directory.Exists(dir)) return [];
        return Directory.GetFiles(dir, "*.json")
                        .Select(Path.GetFileName)
                        .Where(f => f != null)
                        .ToArray()!;
    }

    private static MessageWrapper WrapArcData(string identifier, byte[] data)
    {
        var builder = new FlatBufferBuilder(data.Length + 128);
        var idOffset = builder.CreateString(identifier);
        var dataOffset = ArcData.CreateDataPayloadVector(builder, data);

        ArcData.StartArcData(builder);
        ArcData.AddDataIdentifier(builder, idOffset);
        ArcData.AddDataPayload(builder, dataOffset);
        var offset = ArcData.EndArcData(builder);
        builder.Finish(offset.Value);

        return MessageWrapper.From(MessageType.ArcData, builder.SizedByteArray());
    }
}