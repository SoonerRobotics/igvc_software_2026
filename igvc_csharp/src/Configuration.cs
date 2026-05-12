using System.Net;
using System.Runtime.InteropServices;
using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Core.Units;
using igvc_csharp.Utils;
using OpenCvSharp;
using LogLevel = Microsoft.Extensions.Logging.LogLevel;

namespace igvc_csharp;

public static class Configuration
{
    /// <summary>
    /// Global robot periodic rate (fixed delay)
    /// </summary>
    [Config("robot.periodic_rate")]
    public static readonly TimeSpan PeriodicRate = TimeSpan.FromMilliseconds(1000 / 10);

    /// <summary>
    /// Determines if the robot will use the simulator.
    /// </summary>
    [Config("simulation.enabled")]
    public const bool UseSimulation = true;

    /// <summary>
    /// A magic header for all networking nonsense
    /// </summary>
    [Config("robot.networking_magic")]
    public static readonly byte[] NetworkingMagic = "IGVC"u8.ToArray();

    public static readonly string ChronosOutputDirectory = "~/.scr/chronos";
    
    // Core Constants

    public static class Logging
    {
        /// <summary>
        /// The minimum log level at which the logger will log.
        /// </summary>
        [Config("logging.level")]
        public const LogLevel Level = LogLevel.Trace;
    }

    public static class Config
    {
        /// <summary>
        /// The directory that all presets will be placed in.
        /// <b>NOTE:</b> This will be created if it does not exist.
        /// </summary>
        public const string PresetsDirectory = "~/.igvc/config";
        
        /// <summary>
        /// The default preset name.
        /// <b>NOTE:</b> This will be created if it does not exist.
        /// </summary>
        public const string DefaultPreset = "default";
    }

    public static class Hardware
    {
        public static bool IsLinux = RuntimeInformation.IsOSPlatform(OSPlatform.Linux);
        
        /// <summary>
        /// The name of the interface where the Canbus is connected to.
        /// </summary>
        [Config("hardware.can.interface")]
        public const string CanbusInterface = "can0";
        
        /// <summary>
        /// How often to retry our connection to the Canbus.
        /// </summary>
        [Config("hardware.can.timeout")]
        public static readonly TimeSpan CanbusTimeout = TimeSpan.FromMilliseconds(500);
    }
    
    // Subsystem Constants
    
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
        /// The Port of the simulator, typically 4001.
        /// </summary>
        [Config("simulator.port")]
        public const int Port = 4001;

        /// <summary>
        /// How long between reconnects.
        /// </summary>
        [Config("simulator.reconnect_delay")]
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
        public static readonly ColorUtils.ColorRange GroundThreshold = ColorUtils.ColorRange.From(
            ColorUtils.Color.FromHsv(0,   0,   0),
            ColorUtils.Color.FromHsv(180, 95,  160)
        );

        [Config("vision.yellow_threshold")]
        public static readonly ColorUtils.ColorRange YellowThreshold = ColorUtils.ColorRange.From(
            ColorUtils.Color.FromHsv(15,  80,  80),
            ColorUtils.Color.FromHsv(40,  255, 255)
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
        /// The max forward speed of the robot<br/>
        /// <b>NOTE:</b> This defaults to 5mph as per competition rules
        /// </summary>
        [Config("drive.max_forward")]
        public static readonly LinearVelocity MaxForwardSpeed = LinearVelocityUnit.MilesPerHour.Of(5);

        /// <summary>
        /// The max sideways speed of the robot<br/>
        /// <b>NOTE:</b> This defaults to 5mph as per competition rules
        /// </summary>
        [Config("drive.max_sideways")]
        public static readonly LinearVelocity MaxSidewaysSpeed = LinearVelocityUnit.MilesPerHour.Of(5);

        /// <summary>
        /// The max angular speed of the robot<br/>
        /// <b>NOTE:</b> This defaults to 180 degrees per second (feels like a sane default)
        /// </summary>
        [Config("drive.max_angular")]
        public static readonly AngularVelocity MaxAngularSpeed = AngularVelocityUnit.DegreesPerSecond.Of(30);

        [Config("drive.invert_forward")]
        public static readonly bool InvertForwardVelocity = false;

        [Config("drive.invert_sideways")]
        public static readonly bool InvertSidewaysVelocity = false;

        [Config("drive.invert_angular")]
        public static readonly bool InvertAngularVelocity = false;

        [Config("drive.update_frequency")]
        public static readonly TimeSpan UpdateFrequency = TimeSpan.FromMilliseconds(100);
    }

    public static class CalibrationSubsystem
    {
        /// <summary>
        /// How long to keep the opencv calibration tool active before timing out
        /// </summary>
        public const ulong OpenCvCalibrationTimeoutMs = 60_000;
        
        /// <summary>
        /// The width of the OpenCV calibration pattern (number of inner corners)
        /// </summary>
        public const int OpenCvCalibrationPatternWidth = 7;
        
        /// <summary>
        /// The height of the OpenCV calibration pattern (number of inner corners)
        /// </summary>
        public const int OpenCvCalibrationPatternHeight = 7;
        
        /// <summary>
        /// The size of each square in the OpenCV calibration pattern, in meters.
        /// </summary>
        public const double OpenCvCalibrationSquareSizeMeters = 0.024;
    }
}