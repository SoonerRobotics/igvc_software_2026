using System.Net;
using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Core.Units;
using igvc_csharp.Utilities;
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
        [Config("logging.level")]
        public const LogLevel Level = LogLevel.Trace;
    }

    public static class Configuration
    {
        public const string PresetsDirectory = "~/.igvc/config";
        public const string DefaultPresetName = "default";
    }
    
    public static class ArcSubsystem
    {
        /// <summary>
        /// Whether or not the Arc system should be enabled.
        /// </summary>
        [Config("arc.enabled")]
        public const bool Enabled = true;

        /// <summary>
        /// The host that the ArcServer will bind to
        /// </summary>
        public static readonly IPAddress Host = IPAddress.Any;

        /// <summary>
        /// The port the ArcServer (WebsocketServer) will listen on
        /// </summary>
        [Config("arc.port")]
        public const int Port = 8080;

        /// <summary>
        /// The path the ArcServer will listen on
        /// </summary>
        [Config("arc.path")]
        public const string Path = "/";

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
        [Config("simulator.host")]
        public const string Host = "127.0.0.1";

        /// <summary>
        /// The Port of the simulator, typically 8080.
        /// </summary>
        [Config("simulator.port")]
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

    public static class VisionSubsystem
    {
        /// <summary>
        /// The range of color we are accepting as the "ground"
        /// </summary>
        [Config("vision.ground_threshold")]
        public static readonly ColorUtilities.ColorRange GroundThreshold = ColorUtilities.ColorRange.From(
            ColorUtilities.Color.FromHsv(0, 0, 0),
            ColorUtilities.Color.FromHsv(255, 255, 255)
        );

        /// <summary>
        /// The radius of blurring we apply
        /// </summary>
        [Config("vision.blur_radius")]
        public const int BlurRadius = 5;
        
        /// <summary>
        /// The strength of blurring we apply
        /// </summary>
        [Config("vision.blur_strength")]
        public const int BlurStrength = 3;
    }

    public static class DriveSubsystem
    {
        /// <summary>
        /// The max speed of the robot<br/>
        /// <b>NOTE:</b> This defaults to 5mph as per competition rules
        /// </summary>
        public static readonly LinearVelocity MaxSpeed = LinearVelocityUnit.MilesPerHour.Of(5);

        /// <summary>
        /// The max angular speed of the robot<br/>
        /// <b>NOTE:</b> This defaults to 180 degrees per second (feels like a sane default)
        /// </summary>
        public static readonly AngularVelocity MaxAngularSpeed = AngularVelocityUnit.DegreesPerSecond.Of(180);
    }
}