using Microsoft.Extensions.Logging;

namespace igvc_csharp;

public static class Logging
{
    
    private static ILoggerFactory? _factory;

    private static ILoggerFactory Factory
    {
        get
        {
            if (_factory != null)
            {
                return _factory;
            }
            
            _factory = LoggerFactory.Create(builder =>
            {
                builder.SetMinimumLevel(Constants.Logging.Level);
                builder.AddConsole();
            });
            return _factory;
        }
    }
    
    /// <summary>
    /// Creates a logger for a given class
    /// <code>
    /// private static readonly ILogger Logger = Logging.From&lt;MyClass&gt;();
    /// </code>
    /// 
    /// </summary>
    /// <typeparam name="T">The <c>Class</c> that will do the logging.</typeparam>
    /// <returns>An ILogger instance.</returns>
    public static ILogger From<T>() => Factory.CreateLogger<T>();
    
    /// <summary>
    /// Creates a logger for a given type
    /// </summary>
    /// <param name="type"></param>
    /// <returns></returns>
    public static ILogger From(Type type) => Factory.CreateLogger(type);

    /// <summary>
    ///  Shuts down and disposes the ILoggerFactory
    /// </summary>
    public static void Shutdown()
    {
        _factory?.Dispose();
        _factory = null;
    }
}