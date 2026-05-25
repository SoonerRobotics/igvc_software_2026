using System.Text.Json;
using igvc_csharp.Core.Config;
using igvc_csharp.Utils;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Core.Config;

public class PresetManager
{
    private static readonly ILogger Logger = Logging.From<PresetManager>();
    private static readonly JsonSerializerOptions JsonSerializerOptions = new()
    {
        WriteIndented = true,
        IndentSize = 4
    };

    public static void Initialize()
    {
        var resolvedDir = FileUtils.ExpandPath(Configuration.Config.PresetsDirectory);
        Directory.CreateDirectory(resolvedDir);

        var defaultPath = Path.Combine(resolvedDir, BootParameters.Preset);
        if (!File.Exists(defaultPath))
        {
            WritePreset(defaultPath);
            return;
        }

        LoadPreset(defaultPath);
    }

    public static void LoadPreset(string path)
    {
        var json = File.ReadAllText(path);
        using var doc = JsonDocument.Parse(json);

        foreach (var prop in doc.RootElement.EnumerateObject())
        {
            if (!ConfigManager.Bindings.TryGetValue(prop.Name, out var binding))
            {
                Logger.LogWarning("LoadPreset: unknown key '{Key}', skipping", prop.Name);
                continue;
            }

            object? value;
            try
            {
                value = JsonSerializer.Deserialize(prop.Value.GetRawText(), binding.ValueType);
            }
            catch (Exception ex)
            {
                Logger.LogWarning(ex, "LoadPreset: failed to deserialise '{Key}', skipping", prop.Name);
                continue;
            }

            if (value == null) continue;

            // ConfigManager.Set publishes ConfigChangedEvent, which ArcConfigHandler
            // listens to and broadcasts a config_key_changed message to all Arc clients.
            ConfigManager.Set(prop.Name, value);
        }

        Logger.LogInformation("Loaded preset {Path}", path);
    }

    public static void WritePreset(string path)
    {
        var dict = new Dictionary<string, object>();
        foreach (var (key, binding) in ConfigManager.Bindings)
        {
            dict[key] = binding.Serialize();
        }

        var json = JsonSerializer.Serialize(dict, JsonSerializerOptions);
        File.WriteAllText(path, json);
        Logger.LogInformation("Saved preset {Path}", path);
    }
}