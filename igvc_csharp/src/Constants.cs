using System.Net;
using igvc_csharp.Core;
using Microsoft.Extensions.Logging;

namespace igvc_csharp;
public static class Constants
{
    /// <summary>
    /// Global robot periodic rate (fixed delay)
    /// </summary>
    public static readonly TimeSpan PeriodicRate = TimeSpan.FromMilliseconds(1000 / 10);

    /// <summary>
    /// Determines if the robot will use the simulator.
    /// </summary>
    public const bool UseSimulation = true;

    /// <summary>
    /// A magic header for all networking nonsense
    /// </summary>
    public static readonly byte[] NetworkingMagic = "IGVC"u8.ToArray();
    
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
        /// Whether or not the Arc system should be enabled.
        /// </summary>
        public const bool Enabled = false;
        
        /// <summary>
        /// The host that the ArcServer will bind to
        /// </summary>
        public static readonly IPAddress Host = IPAddress.Loopback;
        
        /// <summary>
        /// The port the ArcServer (WebsocketServer) will listen on
        /// </summary>
        public const int Port = 7000;
        
        /// <summary>
        /// The path the ArcServer will listen on
        /// </summary>
        public const string Path = "";
        
        /// <summary>
        /// The maximum number of clients that can be connected to the ArcServer
        /// </summary>
        public const int MaxConnections = 32;

        /// <summary>
        /// The size of the receiving buffer for incoming messages
        /// </summary>
        public const int ReceiveBufferSize = 4096000;

        /// <summary>
        /// The Endianness of data, both inbound and outbound.
        /// </summary>
        public const Endianness Endianness = Core.Endianness.Little;
    }

    public static class SimulatorSubsystem
    {
        /// <summary>
        /// The Host of the simulator, typically 127.0.0.1.
        /// </summary>
        public const string Host = "127.0.0.1";
        
        /// <summary>
        /// The Port of the simulator, typically 8080.
        /// </summary>
        public const int Port = 4001;

        /// <summary>
        /// How long between reconnects.
        /// </summary>
        public static readonly TimeSpan ReconnectDelay = TimeSpan.FromSeconds(3);
        
        
        /// <summary>
        /// The size of the receiving buffer for incoming messages.
        /// </summary>
        public const int ReceiveBufferSize = 4096000;
        
        /// <summary>
        /// The Endianness of data, both inbound and outbound.
        /// </summary>
        public const Endianness Endianness = Core.Endianness.Little;
    }
}