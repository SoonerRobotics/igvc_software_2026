using System.Net;
using System.Runtime.InteropServices;
using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Core.Units;
using igvc_csharp.Utilities;
using OpenCvSharp;
using LogLevel = Microsoft.Extensions.Logging.LogLevel;

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
    
    // Core Constants

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
        /// <summary>
        /// The directory that all presets will be placed in.
        /// <b>NOTE:</b> This will be created if it does not exist.
        /// </summary>
        public const string PresetsDirectory = "~/.igvc/config";
        
        /// <summary>
        /// The default preset name.
        /// <b>NOTE:</b> This will be created if it does not exist.
        /// </summary>
        public const string DefaultPresetName = "default";
    }

    public static class Experiments
    {
        /// <summary>
        /// Determines if the simulator will use virtual can.<br/>
        /// <b>NOTE:</b> You must be on linux for this experiment to work.
        /// </summary>
        [Config("experiments.simulator.vcan.enabled")]
        public static readonly bool SimulatorUsesVCan = false && Hardware.IsLinux;

        /// <summary>
        /// If the simulator is using virtual can, what interface?
        /// </summary>
        [Config("experiments.simulator.vcan.interface")]
        public const string SimulatorVCanInterface = "vcan0";
    }

    public static class Hardware
    {
        public static bool IsLinux = RuntimeInformation.IsOSPlatform(OSPlatform.Linux);
        
        /// <summary>
        /// The name of the interface where the Canbus is connected to.
        /// </summary>
        public const string CanbusInterface = "can0";
        
        /// <summary>
        /// How often to retry our connection to the Canbus.
        /// </summary>
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
    
    // Competition Constants

    public static class IGVC
    {
        /// <summary>
        /// The minimum distance from the bottom of the stop sign to the ground<br/>
        /// <b>NOTE:</b> This defaults to 5ft per the rules (page 21)
        /// </summary>
        public static readonly Distance StopSignHeight = DistanceUnit.Feet.Of(5);

        /// <summary>
        /// The average diameter of a barrel on the course.<br/>
        /// <b>NOTE:</b> This defaults to 23.5in per the rules (page 21). Although, this may be different for AutoNav.
        /// </summary>
        public static readonly Distance BarrelDiameter = DistanceUnit.Inches.Of(23.5);
    }

    public static class SelfDrive
    {
        /// <summary>
        /// The maximum distance to be from the barrel at the end of a test.<br/>
        /// <b>NOTE:</b> As far as I can tell, this is the same for every test.
        /// </summary>
        public static readonly Distance BarrelStopDistance = DistanceUnit.Feet.Of(3);

        /// <summary>
        /// The distance range at which the robot must change lanes, distance is to the barrel in the same lane.<br/>
        /// <b>NOTE:</b> Defaults of 10ft - 13ft are derived from the rules (page 39).
        /// </summary>
        public static readonly (Distance, Distance) LaneChangeDistance = (DistanceUnit.Feet.Of(10), DistanceUnit.Feet.Of(13));
        
        /// <summary>
        /// The desired speed to travel during a test or run.<br/>
        /// <b>NOTE:</b> The default of 4mph is derived from the middle of the minimum and maximum speed
        /// as noted by the rules (3mph and 5mph respectively).
        /// </summary>
        public static readonly LinearVelocity TargetSpeed = LinearVelocityUnit.MilesPerHour.Of(4);
    }
}