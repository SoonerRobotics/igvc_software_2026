using Microsoft.Extensions.Logging;

namespace igvc_csharp;
public static class Constants
{
    /// <summary>
    /// Global robot periodic rate (fixed delay)
    /// </summary>
    public static readonly TimeSpan PeriodicRate = TimeSpan.FromMilliseconds(1000 / 10);
    public static class Logging
    {
        /// <summary>
        /// The minimum log level at which the logger will log.
        /// </summary>
        public const LogLevel Level = LogLevel.Trace;
    }

    public static class ArcSubsystem
    {
        /// <summary>
        /// The port the ArcServer (WebsocketServer) will listen on
        /// </summary>
        public const int Port = 5805;
        
        /// <summary>
        /// The path the ArcServer will listen on
        /// </summary>
        public const string Path = "/";
        
        /// <summary>
        /// The maximum number of clients that can be connected to the ArcServer
        /// </summary>
        public const int MaxConnections = 32;
    }
}