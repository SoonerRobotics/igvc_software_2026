using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Arc.Config;
using igvc_csharp.Utils.Messages;
using Messages.Arc;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Arc;

/// <summary>
/// Handles all configuration-related Arc commands.
///
/// Responsibilities:
///   • Sends a full config snapshot to every client on connect.
///   • Handles SetConfigKey / LoadPreset / SavePreset / GetPresetList commands.
///   • Listens for ConfigChangedEvent and broadcasts key-changed messages to all Arc clients.
///
/// This is intentionally a separate subsystem so that ArcSubsystem stays
/// focused on transport concerns.
/// </summary>
[Subsystem("ArcConfigHandler", Disabled = !Configuration.ArcSubsystem.Enabled)]
public class ArcConfigHandler : SubsystemBase
{
    // ------------------------------------------------------------------ //
    //  Init / Shutdown                                                     //
    // ------------------------------------------------------------------ //

    public override Task Init(CancellationToken token)
    {
        Subscribe<ConfigChangedEvent>(OnConfigChanged, token);
        Subscribe<ArcClientConnectedEvent>(OnClientConnected, token);

        Logger.LogInformation("ArcConfigHandler initialized");
        SetOperatingState(SubsystemState.Operating);
        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        SetOperatingState(SubsystemState.Shutdown);
        return Task.CompletedTask;
    }

    // ------------------------------------------------------------------ //
    //  Event: new Arc client connected → send full snapshot               //
    // ------------------------------------------------------------------ //

    private async Task OnClientConnected(ArcClientConnectedEvent e, CancellationToken token)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        using var wrapper = ArcConfigMessageFactory.CreateSnapshot(ConfigManager.Bindings);
        await arc.SendToClientWrapper(e.ClientId, wrapper, token);
    }

    // ------------------------------------------------------------------ //
    //  Event: config key changed → broadcast to all Arc clients           //
    // ------------------------------------------------------------------ //

    private async Task OnConfigChanged(ConfigChangedEvent e, CancellationToken token)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        using var wrapper = ArcConfigMessageFactory.CreateKeyChanged(e.Path, e.Value);
        await arc.BroadcastAsync(wrapper, token);
    }

    // ------------------------------------------------------------------ //
    //  Arc Commands                                                        //
    // ------------------------------------------------------------------ //

    /// <summary>
    /// ARC → Robot: request a full config snapshot.
    /// Payload: (none)
    /// </summary>
    [ArcCommand(ArcCommandId.GetConfigSnapshot)]
    public void HandleGetConfigSnapshot(ArcCommand command)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        using var wrapper = ArcConfigMessageFactory.CreateSnapshot(ConfigManager.Bindings);
        _ = arc.SendToClientWrapper(command.SourceClientId, wrapper);
    }

    /// <summary>
    /// ARC → Robot: set a single config key.
    /// Payload (UTF-8): "{path}\0{jsonValue}"
    ///   path      – the dot-separated config path, e.g. "ArcSubsystem.Port"
    ///   jsonValue – JSON representation of the new value (string, number, bool)
    /// </summary>
    [ArcCommand(ArcCommandId.SetConfigKey)]
    public void HandleSetConfigKey(ArcCommand command)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        try
        {
            var text = System.Text.Encoding.UTF8.GetString(command.PayloadBytes);
            var nullIdx = text.IndexOf('\0');
            if (nullIdx < 0)
            {
                Logger.LogWarning("SetConfigKey: malformed payload (no null separator)");
                SendAck(arc, command.SourceClientId, false, "Malformed payload");
                return;
            }

            var path = text[..nullIdx];
            var jsonValue = text[(nullIdx + 1)..];

            if (!ConfigManager.Bindings.TryGetValue(path, out var binding))
            {
                Logger.LogWarning("SetConfigKey: unknown path '{Path}'", path);
                SendAck(arc, command.SourceClientId, false, $"Unknown config key: {path}");
                return;
            }

            // Deserialise the JSON value to the binding's declared type
            var parsed = System.Text.Json.JsonSerializer.Deserialize(jsonValue, binding.ValueType);
            if (parsed == null)
            {
                SendAck(arc, command.SourceClientId, false, "Failed to deserialise value");
                return;
            }

            var ok = ConfigManager.Set(path, parsed);
            SendAck(arc, command.SourceClientId, ok, ok ? null : "TrySet rejected the value");
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "SetConfigKey failed");
            SendAck(arc, command.SourceClientId, false, ex.Message);
        }
    }

    /// <summary>
    /// ARC → Robot: list available presets.
    /// Payload: (none)
    /// </summary>
    [ArcCommand(ArcCommandId.GetPresetList)]
    public void HandleGetPresetList(ArcCommand command)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        using var wrapper = ArcConfigMessageFactory.CreatePresetList();
        _ = arc.SendToClientWrapper(command.SourceClientId, wrapper);
    }

    /// <summary>
    /// ARC → Robot: load a preset by filename.
    /// Payload (UTF-8): preset filename, e.g. "competition.json"
    /// </summary>
    [ArcCommand(ArcCommandId.LoadPreset)]
    public void HandleLoadPreset(ArcCommand command)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        try
        {
            var filename = System.Text.Encoding.UTF8.GetString(command.PayloadBytes).Trim();
            var dir = FileUtils.ExpandPath(Configuration.Config.PresetsDirectory);
            var path = Path.Combine(dir, filename);

            // Basic path-traversal guard
            if (!Path.GetFullPath(path).StartsWith(Path.GetFullPath(dir), StringComparison.Ordinal))
            {
                SendAck(arc, command.SourceClientId, false, "Invalid preset path");
                return;
            }

            if (!File.Exists(path))
            {
                SendAck(arc, command.SourceClientId, false, $"Preset not found: {filename}");
                return;
            }

            PresetManager.LoadPreset(path);
            Logger.LogInformation("Loaded preset '{Filename}' via Arc", filename);

            // The individual ConfigChangedEvents fired by LoadPreset will
            // propagate each key to all clients automatically.
            // Still send an ack so the frontend can update its "current preset" label.
            SendAck(arc, command.SourceClientId, true, filename);
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "LoadPreset failed");
            SendAck(arc, command.SourceClientId, false, ex.Message);
        }
    }

    /// <summary>
    /// ARC → Robot: save the current config as a preset.
    /// Payload (UTF-8): preset filename, e.g. "my_preset.json"
    /// </summary>
    [ArcCommand(ArcCommandId.SavePreset)]
    public void HandleSavePreset(ArcCommand command)
    {
        var arc = BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();
        if (arc == null) return;

        try
        {
            var filename = System.Text.Encoding.UTF8.GetString(command.PayloadBytes).Trim();

            // Enforce .json extension so GetPresetNames() picks it up
            if (!filename.EndsWith(".json", StringComparison.OrdinalIgnoreCase))
                filename += ".json";

            var dir = FileUtils.ExpandPath(Configuration.Config.PresetsDirectory);
            var path = Path.Combine(dir, filename);

            // Basic path-traversal guard
            if (!Path.GetFullPath(path).StartsWith(Path.GetFullPath(dir), StringComparison.Ordinal))
            {
                SendAck(arc, command.SourceClientId, false, "Invalid preset path");
                return;
            }

            PresetManager.WritePreset(path);
            Logger.LogInformation("Saved preset '{Filename}' via Arc", filename);
            SendAck(arc, command.SourceClientId, true, filename);
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "SavePreset failed");
            SendAck(arc, command.SourceClientId, false, ex.Message);
        }
    }

    // ------------------------------------------------------------------ //
    //  Private helpers                                                     //
    // ------------------------------------------------------------------ //

    private static void SendAck(ArcSubsystem arc, Guid clientId, bool success, string? message)
    {
        using var wrapper = ArcConfigMessageFactory.CreateAck(success, message);
        _ = arc.SendToClientWrapper(clientId, wrapper);
    }
}