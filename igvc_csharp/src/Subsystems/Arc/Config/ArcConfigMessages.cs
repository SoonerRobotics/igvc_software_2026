using igvc_csharp.Core.Config;
using igvc_csharp.Utils.Messages;
using System.Text.Json;
using Google.FlatBuffers;
using Messages.Arc;
using igvc_csharp.Utils;

namespace igvc_csharp.Subsystems.Arc.Config;

// ---------------------------------------------------------------------------
//  Wire identifiers – these are the "identifier" strings used in ArcData
//  wrappers, matching whatever your frontend will key on.
// ---------------------------------------------------------------------------
public static class ArcConfigIdentifiers
{
    /// <summary>Full snapshot sent on connect: all keys + all preset names.</summary>
    public const string ConfigSnapshot = "config_snapshot";

    /// <summary>Sent whenever a single key changes (robot → ARC).</summary>
    public const string ConfigKeyChanged = "config_key_changed";

    /// <summary>Response to a list-presets request.</summary>
    public const string ConfigPresetList = "config_preset_list";

    /// <summary>Confirmation / error after a set / save / load command.</summary>
    public const string ConfigAck = "config_ack";
}

// ---------------------------------------------------------------------------
//  ARC → Robot command IDs (add these to your ArcCommandId enum / proto)
// ---------------------------------------------------------------------------
// ArcCommandId.GetConfigSnapshot   – robot sends the full snapshot
// ArcCommandId.SetConfigKey        – payload: key\0value (utf-8 strings, null-separated)
// ArcCommandId.GetPresetList       – robot replies with preset names
// ArcCommandId.LoadPreset          – payload: preset filename (utf-8)
// ArcCommandId.SavePreset          – payload: preset filename (utf-8)

// ---------------------------------------------------------------------------
//  Helper: build ArcData MessageWrappers for config payloads
// ---------------------------------------------------------------------------
public static class ArcConfigMessageFactory
{
    // ------------------------------------------------------------------ //
    //  config_snapshot                                                     //
    // ------------------------------------------------------------------ //
    /// <summary>
    /// Serialises every registered config key → current value, plus the
    /// list of available preset filenames, into a single JSON ArcData blob.
    /// </summary>
    public static MessageWrapper CreateSnapshot(IReadOnlyDictionary<string, IConfigBinding> bindings)
    {
        var keys = new Dictionary<string, object?>();
        foreach (var (path, binding) in bindings)
            keys[path] = binding.Serialize();

        var presets = GetPresetNames();

        var payload = JsonSerializer.SerializeToUtf8Bytes(new
        {
            keys,
            presets,
            currentPreset = BootParameters.Preset
        });

        return WrapArcData(ArcConfigIdentifiers.ConfigSnapshot, payload);
    }

    // ------------------------------------------------------------------ //
    //  config_key_changed                                                  //
    // ------------------------------------------------------------------ //
    public static MessageWrapper CreateKeyChanged(string path, object? value)
    {
        var payload = JsonSerializer.SerializeToUtf8Bytes(new { path, value });
        return WrapArcData(ArcConfigIdentifiers.ConfigKeyChanged, payload);
    }

    // ------------------------------------------------------------------ //
    //  config_preset_list                                                  //
    // ------------------------------------------------------------------ //
    public static MessageWrapper CreatePresetList()
    {
        var payload = JsonSerializer.SerializeToUtf8Bytes(new
        {
            presets = GetPresetNames(),
            currentPreset = BootParameters.Preset
        });
        return WrapArcData(ArcConfigIdentifiers.ConfigPresetList, payload);
    }

    // ------------------------------------------------------------------ //
    //  config_ack                                                          //
    // ------------------------------------------------------------------ //
    public static MessageWrapper CreateAck(bool success, string? message = null)
    {
        var payload = JsonSerializer.SerializeToUtf8Bytes(new { success, message });
        return WrapArcData(ArcConfigIdentifiers.ConfigAck, payload);
    }

    // ------------------------------------------------------------------ //
    //  Internal helpers                                                    //
    // ------------------------------------------------------------------ //
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