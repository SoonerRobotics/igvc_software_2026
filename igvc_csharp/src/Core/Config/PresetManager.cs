using System.Text.Json;
using igvc_csharp.Utils;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Core.Config;

public class PresetManager
{
    private static readonly ILogger Logger = Logging.From<PresetManager>();

    private static readonly JsonSerializerOptions JsonSerializerOptions = new();
    
    public static void Initialize()
    {
        var resolvedDir = FileUtils.ExpandPath(Constants.Configuration.PresetsDirectory);
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

        var json = JsonSerializer.Serialize(dict, JsonSerializerOptions);
        File.WriteAllText(path, json);
        Logger.LogInformation("Saved preset {Path}", path);
    }
}