using System.Text.Json;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Core.Config;

public class PresetManager
{
    private static readonly ILogger Logger = Logging.From<PresetManager>();

    public static void Initialize()
    {
        Directory.CreateDirectory(Constants.Configuration.PresetsDirectory);
        var defaultPath = Path.Combine(Constants.Configuration.PresetsDirectory,
            Constants.Configuration.DefaultPresetName);
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
            ConfigManager.Set(prop.Name, prop.Value);
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

        var json = JsonSerializer.Serialize(
            dict,
            new JsonSerializerOptions
            {
                WriteIndented = true
            });

        File.WriteAllText(path, json);
        Logger.LogInformation("Saved preset {Path}", path);
    }
}