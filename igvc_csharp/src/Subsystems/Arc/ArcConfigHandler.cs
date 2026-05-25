using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Arc.Config;
using igvc_csharp.Utils;
using Messages.Arc;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Arc;

[Subsystem("ArcConfigHandler", Disabled = !Configuration.ArcSubsystem.Enabled)]
public class ArcConfigHandler : SubsystemBase
{
    private static readonly Microsoft.Extensions.Logging.ILogger Log = Logging.From<ArcConfigHandler>();

    public override Task Init(CancellationToken token)
    {
        Subscribe<ConfigChangedEvent>(OnConfigChanged, token);
        Subscribe<ArcClientConnectedEvent>(OnClientConnected, token);

        Log.LogInformation("ArcConfigHandler initialized");
        SetOperatingState(SubsystemState.Operating);
        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        SetOperatingState(SubsystemState.Shutdown);
        return Task.CompletedTask;
    }

    // ── Events ────────────────────────────────────────────────────────────────

    private async Task OnClientConnected(ArcClientConnectedEvent e, CancellationToken token)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        using var wrapper = ArcConfigMessageFactory.CreateSnapshot(ConfigManager.Bindings);
        await arc.SendToClientWrapper(e.ClientId, wrapper, token);
    }

    private async Task OnConfigChanged(ConfigChangedEvent e, CancellationToken token)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        var serialized = ConfigSerializer.Serialize(e.Value);
        using var wrapper = ArcConfigMessageFactory.CreateKeyChanged(e.Path, serialized);
        await arc.BroadcastAsync(wrapper, token);
    }

    // ── Arc Commands ──────────────────────────────────────────────────────────

    [ArcCommand(ArcCommandId.GetConfigSnapshot)]
    public static void HandleGetConfigSnapshot(ArcCommandContext ctx)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        using var wrapper = ArcConfigMessageFactory.CreateSnapshot(ConfigManager.Bindings);
        _ = arc.SendToClientWrapper(ctx.ClientId, wrapper);
    }

    /// <summary>
    /// SetConfigKey wire format (matches config.ts encodeSetConfigKey):
    ///   [2 bytes: uint16 LE path byte length][path UTF-8][value JSON UTF-8]
    /// </summary>
    [ArcCommand(ArcCommandId.SetConfigKey)]
    public static void HandleSetConfigKey(ArcCommandContext ctx)
    {
        Log.LogInformation("HandleSetConfigKey called");

        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        try
        {
            var data = ctx.Command.GetDataArray() ?? [];

            if (data.Length < 2)
            {
                Log.LogWarning("SetConfigKey: payload too short");
                SendAck(arc, ctx.ClientId, false, "Payload too short");
                return;
            }

            // Read uint16 LE path length
            var pathLen = data[0] | (data[1] << 8);

            if (data.Length < 2 + pathLen)
            {
                Log.LogWarning("SetConfigKey: path length {PathLen} exceeds payload size {DataLen}", pathLen, data.Length);
                SendAck(arc, ctx.ClientId, false, "Malformed payload");
                return;
            }

            var path = System.Text.Encoding.UTF8.GetString(data, 2, pathLen);
            var jsonValue = System.Text.Encoding.UTF8.GetString(data, 2 + pathLen, data.Length - 2 - pathLen);

            if (!ConfigManager.Bindings.TryGetValue(path, out var binding))
            {
                Log.LogWarning("SetConfigKey: unknown path '{Path}'", path);
                SendAck(arc, ctx.ClientId, false, $"Unknown config key: {path}");
                return;
            }

            object? parsed;
            try
            {
                parsed = ConfigSerializer.Deserialize(jsonValue, binding.ValueType);
            }
            catch (Exception ex)
            {
                Log.LogWarning(ex, "SetConfigKey: failed to deserialise value for '{Path}'", path);
                SendAck(arc, ctx.ClientId, false, $"Failed to deserialise value: {ex.Message}");
                return;
            }

            if (parsed == null)
            {
                SendAck(arc, ctx.ClientId, false, "Deserialised value was null");
                return;
            }

            var ok = ConfigManager.Set(path, parsed);
            SendAck(arc, ctx.ClientId, ok, ok ? null : "TrySet rejected the value");
        }
        catch (Exception ex)
        {
            Log.LogError(ex, "SetConfigKey failed");
            SendAck(arc, ctx.ClientId, false, ex.Message);
        }
    }

    [ArcCommand(ArcCommandId.GetPresetList)]
    public static void HandleGetPresetList(ArcCommandContext ctx)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        using var wrapper = ArcConfigMessageFactory.CreatePresetList();
        _ = arc.SendToClientWrapper(ctx.ClientId, wrapper);
    }

    [ArcCommand(ArcCommandId.LoadPreset)]
    public static void HandleLoadPreset(ArcCommandContext ctx)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        try
        {
            var filename = System.Text.Encoding.UTF8.GetString(ctx.Command.GetDataArray() ?? []).Trim();
            var dir = FileUtils.ExpandPath(Configuration.Config.PresetsDirectory);
            var path = Path.Combine(dir, filename);

            if (!Path.GetFullPath(path).StartsWith(Path.GetFullPath(dir), StringComparison.Ordinal))
            {
                SendAck(arc, ctx.ClientId, false, "Invalid preset path");
                return;
            }

            if (!File.Exists(path))
            {
                SendAck(arc, ctx.ClientId, false, $"Preset not found: {filename}");
                return;
            }

            PresetManager.LoadPreset(path);
            Log.LogInformation("Loaded preset '{Filename}' via Arc", filename);
            SendAck(arc, ctx.ClientId, true, filename);
        }
        catch (Exception ex)
        {
            Log.LogError(ex, "LoadPreset failed");
            SendAck(arc, ctx.ClientId, false, ex.Message);
        }
    }

    [ArcCommand(ArcCommandId.SavePreset)]
    public static void HandleSavePreset(ArcCommandContext ctx)
    {
        Log.LogInformation("HandleSavePreset called");

        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        try
        {
            var filename = System.Text.Encoding.UTF8.GetString(ctx.Command.GetDataArray() ?? []).Trim();

            if (!filename.EndsWith(".json", StringComparison.OrdinalIgnoreCase))
                filename += ".json";

            var dir = FileUtils.ExpandPath(Configuration.Config.PresetsDirectory);
            var path = Path.Combine(dir, filename);

            if (!Path.GetFullPath(path).StartsWith(Path.GetFullPath(dir), StringComparison.Ordinal))
            {
                SendAck(arc, ctx.ClientId, false, "Invalid preset path");
                return;
            }

            PresetManager.WritePreset(path);
            Log.LogInformation("Saved preset '{Filename}' via Arc", filename);
            SendAck(arc, ctx.ClientId, true, filename);
        }
        catch (Exception ex)
        {
            Log.LogError(ex, "SavePreset failed");
            SendAck(arc, ctx.ClientId, false, ex.Message);
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private static void SendAck(ArcSubsystem arc, Guid clientId, bool success, string? message)
    {
        using var wrapper = ArcConfigMessageFactory.CreateAck(success, message);
        _ = arc.SendToClientWrapper(clientId, wrapper);
    }
}