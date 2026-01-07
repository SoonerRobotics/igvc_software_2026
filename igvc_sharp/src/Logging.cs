using Microsoft.Extensions.Logging;

namespace igvc_sharp;

public class Logging
{
    private static readonly ILoggerFactory Factory = LoggerFactory.Create(builder => 
        // Outputs
        builder.AddConsole()
        
        // Config
        .AddFilter("*", Constants.DefaultLogLevel)
        );
    
    public static ILogger Create(string name)
    {
        return Factory.CreateLogger(name);
    }

    public static ILogger ForContext<T>()
    {
        return Factory.CreateLogger<T>();
    }

    public static void Dispose()
    {
        Factory.Dispose();
    }
}