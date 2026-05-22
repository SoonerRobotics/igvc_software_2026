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
    public const bool UseSimulation = false;

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
            ColorUtils.Color.FromHsv(0, 0, 0),
            ColorUtils.Color.FromHsv(180, 95, 160)
        );

        [Config("vision.yellow_threshold")]
        public static readonly ColorUtils.ColorRange YellowThreshold = ColorUtils.ColorRange.From(
            ColorUtils.Color.FromHsv(15, 80, 80),
            ColorUtils.Color.FromHsv(40, 255, 255)
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
        public static readonly AngularVelocity MaxAngularSpeed = AngularVelocityUnit.DegreesPerSecond.Of(200);

        [Config("drive.invert_forward")]
        public static readonly bool InvertForwardVelocity = false;

        [Config("drive.invert_sideways")]
        public static readonly bool InvertSidewaysVelocity = true;

        [Config("drive.invert_angular")]
        public static readonly bool InvertAngularVelocity = true;

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

    public static class FeelerSubsystem
    {
        // feeler-related config
        /// <summary>
        /// Whether to use feelers or not (as opposed to, e.x. the A* subsystem)
        /// </summary>
        [Config("feelers.use_feelers")]
        public const bool UseFeelers = true;

        /// <summary>
        /// Default (unbiased) max length of the vision feelers, in pixels
        /// </summary>
        [Config("feelers.max_length")]
        public const int MaxLength = 100;

        /// <summary>
        /// The starting angle of the 1st feeler, in degrees. 0 degrees is the positive x-axis
        /// </summary>
        [Config("feelers.angle_offset")]
        public const double AngleOffset = 20;

        /// <summary>
        /// How large the arc of feelers is, in degrees (determines where the last feeler is placed)
        /// </summary>
        [Config("feelers.angular_width")]
        public const double AngularWidth = 180 - 20;

        /// <summary>
        /// How many feelers to make across the specified arc.
        /// </summary>
        [Config("feelers.num_feelers")]
        public const int NumFeelers = 24;

        /// <summary>
        /// TODO
        /// </summary>
        [Config("feelers.balace_feelers")]
        public const bool BalanceFeelers = false;

        /// <summary>
        /// Whether to use only GPS waypoints and ignore obstacles or not
        /// </summary>
        [Config("feelers.use_only_waypoints")]
        public const bool UseOnlWaypoints = false;

        /// <summary>
        /// How long to wait, after starting the run, before factoring in the GPS waypoints, in milliseconds
        /// </summary>
        [Config("feelers.gps_wait_time")]
        public const ulong GpsWaitTime = 1000 * 30;

        /// <summary>
        /// The maximum amount each feeler should be biased by the GPS feeler, in pixels
        /// </summary>
        [Config("feelers.gps_bias_weight")]
        public const int GpsBias = 75;

        /// <summary>
        /// The maximum amount each feeler should be biased by the forward feeler, in pixels
        /// </summary>
        [Config("feelers.forward_bias_weight")]
        public const int ForwardBiasWeight = 75;


        // control-related config
        /// <summary>
        /// The maximum forward velocity that feelers is allowed to command, in meters per second
        /// </summary>
        [Config("feelers.max_drive_speed")]
        public const double MaxDriveSpeed = 5;

        /// <summary>
        /// The maximum sideways velocity that feelers is allowed to command, in meters per second
        /// </summary>
        [Config("feelers.max_strafe_speed")]
        public const double MaxStrafeSpeed = 5;

        /// <summary>
        /// The maximum rotational velocity that feelers is allowed to command, in radians per second
        /// </summary>
        [Config("feelers.max_turn_speed")]
        public const double MaxTurnSpeed = 5;

        /// <summary>
        /// The proportional constant for the drive PID
        /// </summary>
        [Config("feelers.drive_kp")]
        public const double DriveKp = 0.001;

        /// <summary>
        /// The integral constant for the drive PID
        /// </summary>
        [Config("feelers.drive_ki")]
        public const double DriveKi = 0.0;

        /// <summary>
        /// The derivative constant for the drive PID
        /// </summary>
        [Config("feelers.drive_kd")]
        public const double DriveKd = 0.0001;

        /// <summary>
        /// The proportional constant for the heading PID
        /// </summary>
        [Config("feelers.heading_kp")]
        public const double HeadingKp = 0.001;

        /// <summary>
        /// The integral constant for the heading PID
        /// </summary>
        [Config("feelers.heading_ki")]
        public const double HeadingKi = 0.0;

        /// <summary>
        /// The derivative constant for the heading PID
        /// </summary>
        [Config("feelers.heading_kd")]
        public const double HeadingKd = 0.0001;
    }

    public static class WaypointSubsystem
    {
        // gps-related config
        /// <summary> 
        /// filename for the waypoints (should be CSV file with label,lat,lon,)
        /// </summary>
        public const string WaypointsFilename = "resources/waypoints.csv";

        /// <summary>
        /// How close we have to be for a GPS waypoint to be considered 'reached,' in meters
        /// </summary>
        [Config("waypoints.waypoint_pop_dist")]
        public const double WaypointPopDist = 1.5;

        /// <summary>
        /// How long we have to be within the WaypointPopDist, in milliseconds
        /// </summary>
        [Config("waypoints.waypoint_pop_time")]
        public const ulong WaypointPopTime = 500;

        /// <summary>
        /// How long to wait, after starting the run, before factoring in the GPS waypoints, in milliseconds
        /// </summary>
        [Config("waypoints.gps_wait_time")]
        public const ulong GpsWaitTime = 1000 * 2;

        /// <summary>
        /// Longitude of the west-most edge of the practice autonav field
        /// </summary>
        public const double PracticeLongitude = -83.218909;

        /// <summary>
        /// Longitude of the west-most edge of the autonav competition field
        /// </summary>
        public const double AutonavLongitude = -83.219584;

        /// <summary>
        /// Longitude of the west-most edge of the selfdrive course
        /// </summary>
        public const double SelfdriveLongitude = -83.217515;

        /// <summary>
        /// Latitude of the northern edge of the engineering quadrangle on OU campus.
        /// </summary>
        public const double EquadLatitude = 35.211160;
    }

    public static class FakeCameraSubsystemConfig
    {
        /// <summary>
        /// Filename of video to send as raw camera frames
        /// </summary>
        public const string Filename = "resources/video/camera.mp4";

        /// <summary>
        /// Frames to publish per second
        /// </summary>
        public const double FPS = 15;
    }

    public static class FakeGpsSubsystem
    {
        public const string Filename = "resources/gps/ENTRY_GPS.csv";
    }
}
