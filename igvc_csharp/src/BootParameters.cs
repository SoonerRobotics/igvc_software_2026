using System.CommandLine;
using Microsoft.Extensions.Logging;

namespace igvc_csharp;

public class BootParameters
{
    private static readonly ILogger Logger = Logging.From<BootParameters>();

    // Preset Option
    public static string Preset { get; private set; } = Configuration.Config.DefaultPreset;
    private static readonly Option<string> PresetFileOption = new("--file")
    {
        Description = "The preset to use",
        DefaultValueFactory = _ => Configuration.Config.DefaultPreset
    };

    private static readonly RootCommand Command = new("IGVC 2026 | Suspended Disbelief")
    {
        PresetFileOption
    };
    
    public static bool Initialize(string[] args)
    {
        var result = Command.Parse(args);
        result.Invoke();
        
        // Check if args included any help symbols
        var symbols = new string[] { "--help", "-help", "-h", "/h", "-?", "/?" };
        if (args.Any(x => symbols.Any(y => y == x)))
        {
            return false;
        }
        
        if (result.Errors.Count > 0)
        {
            foreach (var error in result.Errors)
            {
                Console.Error.WriteLine(error.Message);
            }

            return false;
        }
        
        if (result.GetValue(PresetFileOption) is { } preset)
        {
            Logger.LogInformation("Using preset {Preset}", preset);
            Preset = preset;
        }
        
        return true;
    }
}