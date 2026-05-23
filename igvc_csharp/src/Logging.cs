using igvc_csharp.Core;
using igvc_csharp.Core.Chronos;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Utils;
using Microsoft.Extensions.Logging;

namespace igvc_csharp;

public static class Logging
{

    private static ILoggerFactory? _factory;
    private static AbstractChronosSubsystem? _chronos;
    private static ArcSubsystem? _arc;

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
                builder.SetMinimumLevel(Configuration.Logging.Level);
                // builder.AddSimpleConsole(options =>
                // {
                //     options.SingleLine = true;
                //     options.TimestampFormat = "HH:mm:ss ";
                // });
                builder.AddConsole();
                builder.AddProvider(new LogInterceptorProvider((cat, level, id, message, ex) =>
                {
                    // Try and get the Chronos/Arc instance
                    _chronos ??= BaseRobot.Instance?.GetSubsystem<AbstractChronosSubsystem>();
                    _arc ??= BaseRobot.Instance?.GetSubsystem<ArcSubsystem>();

                    if (_chronos != null)
                    {
                        // Cat (string), Level (byte), EventId (int), EventName (string), Message (string), ExceptionExists (bool), ExceptionMessage (string)
                        var ms = new MemoryStream();
                        using var bw = new BinaryWriter(ms);
                        bw.Write(cat);
                        bw.Write((byte)level);
                        bw.Write(id.Id);
                        bw.Write(id.Name ?? string.Empty);
                        bw.Write(message);
                        bw.Write(ex != null);
                        if (ex != null)
                        {
                            bw.Write(ex.ToString());
                        }
                        _chronos?.WriteEntry(EntryTypeId.SessionLog, ms.ToArray());
                    }

                    if (_arc != null)
                    {
                        var msg = ArcUtils.CreateArcData_Log(
                            cat,
                            (byte)level,
                            id.Id,
                            id.Name ?? string.Empty,
                            message
                        );
                        _arc.BroadcastAsync(msg, CancellationToken.None).ConfigureAwait(false);
                    }
                }));
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