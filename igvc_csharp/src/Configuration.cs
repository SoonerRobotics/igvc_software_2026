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
    public static TimeSpan PeriodicRate = TimeSpan.FromMilliseconds(1000 / 10);

    /// <summary>
    /// Determines if the robot will use the simulator.
    public const bool UseSimulation = false;

    /// <summary>
    /// A magic header for all networking nonsense
    /// </summary>
    [Config("robot.networking_magic")]
    public static byte[] NetworkingMagic = "IGVC"u8.ToArray();

    public static string ChronosOutputDirectory = "~/.igvc/chronos";

    // Core staticants

    public static class Logging
    {
        /// <summary>
        /// The minimum log level at which the logger will log.
        /// </summary>
        [Config("logging.level")]
        public static LogLevel Level = LogLevel.Trace;
    }

    public static class Config
    {
        /// <summary>
        /// The directory that all presets will be placed in.
        /// <b>NOTE:</b> This will be created if it does not exist.
        /// </summary>
        public static string PresetsDirectory = "~/.igvc/config";

        /// <summary>
        /// The default preset name.
        /// <b>NOTE:</b> This will be created if it does not exist.
        /// </summary>
        public static string DefaultPreset = "default";
    }

    public static class Hardware
    {
        public static bool IsLinux = RuntimeInformation.IsOSPlatform(OSPlatform.Linux);
    }

    // Subsystem staticants

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
        public static IPAddress Host = IPAddress.Any;

        /// <summary>
        /// The port the ArcServer (WebsocketServer) will listen on
        /// </summary>
        [Config("arc.port")]
        public static int Port = 8080;

        /// <summary>
        /// The path the ArcServer will listen on
        /// </summary>
        [Config("arc.path")]
        public static string Path = "/";

        /// <summary>
        /// The size of the receiving buffer for incoming messages
        /// </summary>
        public static int ReceiveBufferSize = 4096000;

        /// <summary>
        /// The Endianness of data, both inbound and outbound.
        /// </summary>
        public const Endianness Endianness = Endianness.Little;
    }

    public static class SimulatorSubsystem
    {
        /// <summary>
        /// The Host of the simulator, typically 127.0.0.1.
        /// </summary>
        [Config("simulator.host")]
        public static string Host = "127.0.0.1";

        /// <summary>
        /// The Port of the simulator, typically 4001.
        /// </summary>
        [Config("simulator.port")]
        public static int Port = 4001;

        /// <summary>
        /// How long between reconnects.
        /// </summary>
        [Config("simulator.reconnect_delay")]
        public static TimeSpan ReconnectDelay = TimeSpan.FromSeconds(3);


        /// <summary>
        /// The size of the receiving buffer for incoming messages.
        /// </summary>
        public static int ReceiveBufferSize = 4096000;

        /// <summary>
        /// The Endianness of data, both inbound and outbound.
        /// </summary>
        public const Endianness Endianness = Endianness.Little;
    }

    public static class VisionSubsystem
    {
        /// <summary>
        /// The range of color we are accepting as the "ground"
        /// </summary>
        [Config("vision.ground_threshold")]
        public static ColorUtils.ColorRange GroundThreshold = ColorUtils.ColorRange.From(
            ColorUtils.Color.FromHsv(0, 0, 0),
            ColorUtils.Color.FromHsv(179, 200, 190)
        );

        [Config("vision.yellow_threshold")]
        public static ColorUtils.ColorRange YellowThreshold = ColorUtils.ColorRange.From(
            ColorUtils.Color.FromHsv(15, 80, 80),
            ColorUtils.Color.FromHsv(40, 255, 255)
        );

        /// <summary>
        /// The radius of blurring we apply
        /// </summary>
        [Config("vision.blur_radius")]
        public static int BlurRadius = 5;

        /// <summary>
        /// The strength of blurring we apply
        /// </summary>
        [Config("vision.blur_strength")]
        public static int BlurStrength = 3;

        //FIXME make flattening configurable from the GUI
        // For flattening, the order is [ TL, TR, BR, BL ]

        /// <summary>
        /// The source points for the left camera image flattening
        /// </summary>
        public static Point2f[] leftSourcePoints =
        [
            new(60, 100), //TODO: since these are the only 2 points actually modified, do something?
            new(640 - 90, 100), // same here
            new(640 - 90, 480),
            new(0, 480),
        ];

        /// <summary>
        /// The destination points for the left camera image flattening
        /// </summary>
        public static Point2f[] leftDestPoints =
        [
            new(0, 0),
            new(640, 0),
            new(640, 480), // same here
            new(0, 480), // same here
        ];

        /// <summary>
        /// The source points for the right camera image flattening
        /// </summary>
        public static Point2f[] rightSourcePoints =
        [
            new(0 + 60, 100),     // same here
            new(470, 100),   // same here
            new(640, 480),
            new(0 + 60, 480),
        ];

        /// <summary>
        /// The destination points for the right camera image flattening
        /// </summary>
        public static Point2f[] rightDestPoints =
        [
            new(0, 0),
            new(640, 0),
            new(640, 480),   // same here
            new(0, 480),   // same here
        ];
    }

    public static class DriveSubsystem
    {
        /// <summary>
        /// The max forward speed of the robot<br/>
        /// <b>NOTE:</b> This defaults to 5mph as per competition rules
        /// </summary>
        [Config("drive.max_forward")]
        public static LinearVelocity MaxForwardSpeed = LinearVelocityUnit.MetersPerSecond.Of(1f);

        /// <summary>
        /// The max sideways speed of the robot<br/>
        /// <b>NOTE:</b> This defaults to 5mph as per competition rules
        /// </summary>
        [Config("drive.max_sideways")]
        public static LinearVelocity MaxSidewaysSpeed = LinearVelocityUnit.MetersPerSecond.Of(1f);

        /// <summary>
        /// The max angular speed of the robot<br/>
        /// <b>NOTE:</b> This defaults to 180 degrees per second (feels like a sane default)
        /// </summary>
        [Config("drive.max_angular")]
        public static AngularVelocity MaxAngularSpeed = AngularVelocityUnit.DegreesPerSecond.Of(120);

        [Config("drive.invert_forward")]
        public static bool InvertForwardVelocity = false;

        [Config("drive.invert_sideways")]
        public static bool InvertSidewaysVelocity = true;

        [Config("drive.invert_angular")]
        public static bool InvertAngularVelocity = true;

        [Config("drive.update_frequency")]
        public static TimeSpan UpdateFrequency = TimeSpan.FromMilliseconds(1000 / 50);
    }

    public static class CalibrationSubsystem
    {
        /// <summary>
        /// How long to keep the opencv calibration tool active before timing out
        /// </summary>
        public static ulong OpenCvCalibrationTimeoutMs = 60_000;

        /// <summary>
        /// The width of the OpenCV calibration pattern (number of inner corners)
        /// </summary>
        public static int OpenCvCalibrationPatternWidth = 7;

        /// <summary>
        /// The height of the OpenCV calibration pattern (number of inner corners)
        /// </summary>
        public static int OpenCvCalibrationPatternHeight = 7;

        /// <summary>
        /// The size of each square in the OpenCV calibration pattern, in meters.
        /// </summary>
        public static double OpenCvCalibrationSquareSizeMeters = 0.024;
    }

    public static class FeelerSubsystem
    {
        // feeler-related config
        /// <summary>
        /// Whether to use feelers or not (as opposed to, e.x. the A* subsystem)
        /// </summary>
        [Config("feelers.use_feelers")]
        public static bool UseFeelers = true;

        /// <summary>
        /// Default (unbiased) max length of the vision feelers, in pixels
        /// </summary>
        [Config("feelers.max_length")]
        public static int MaxLength = 250;

        /// <summary>
        /// The starting angle of the 1st feeler, in degrees. 0 degrees is the positive x-axis
        /// </summary>
        [Config("feelers.angle_offset")]
        public static double AngleOffset = 11;

        /// <summary>
        /// How large the arc of feelers is, in degrees (determines where the last feeler is placed)
        /// </summary>
        [Config("feelers.angular_width")]
        public static double AngularWidth = 180 - (AngleOffset * 2);

        /// <summary>
        /// How many feelers to make across the specified arc.
        /// </summary>
        [Config("feelers.num_feelers")]
        public static int NumFeelers = 32;

        /// <summary>
        /// TODO
        /// </summary>
        [Config("feelers.balace_feelers")]
        public static bool BalanceFeelers = false;

        /// <summary>
        /// Whether to use only GPS waypoints and ignore obstacles or not
        /// </summary>
        [Config("feelers.use_only_waypoints")]
        public static bool UseOnlyWaypoints = false;

        /// <summary>
        /// How long to wait, after starting the run, before factoring in the GPS waypoints, in milliseconds
        /// </summary>
        [Config("feelers.gps_wait_time")]
        public static ulong GpsWaitTime = 1000 * 30;

        /// <summary>
        /// The maximum amount each feeler should be biased by the GPS feeler, in pixels
        /// </summary>
        [Config("feelers.gps_bias_weight")]
        public static int GpsBias = 75;

        /// <summary>
        /// The maximum amount each feeler should be biased by the forward feeler, in pixels
        /// </summary>
        [Config("feelers.forward_bias_weight")]
        public static int ForwardBiasWeight = 5;

        /// <summary>
        /// How far up the image to center the feelers from, as a percentage.
        /// Ex. 0.5 would be dead center.
        /// </summary>
        [Config("feelers.y_percentage")]
        public static double YPercentage = 0.9;


        // control-related config
        /// <summary>
        /// The maximum forward velocity that feelers is allowed to command, in meters per second
        /// </summary>
        [Config("feelers.max_drive_speed")]
        public static double MaxDriveSpeed = 1;

        /// <summary>
        /// The maximum sideways velocity that feelers is allowed to command, in meters per second
        /// </summary>
        [Config("feelers.max_strafe_speed")]
        public static double MaxStrafeSpeed = 1;

        /// <summary>
        /// The maximum rotational velocity that feelers is allowed to command, in radians per second
        /// </summary>
        [Config("feelers.max_turn_speed")]
        public static double MaxTurnSpeed = 1;

        /// <summary>
        /// The proportional staticant for the drive PID
        /// </summary>
        [Config("feelers.drive_kp")]
        public static double DriveKp = 0.0005;

        /// <summary>
        /// The integral staticant for the drive PID
        /// </summary>
        [Config("feelers.drive_ki")]
        public static double DriveKi = 0.0;

        /// <summary>
        /// The derivative staticant for the drive PID
        /// </summary>
        [Config("feelers.drive_kd")]
        public static double DriveKd = 0.0;

        /// <summary>
        /// The proportional staticant for the heading PID
        /// </summary>
        [Config("feelers.heading_kp")]
        public static double HeadingKp = 0.003;

        /// <summary>
        /// The integral staticant for the heading PID
        /// </summary>
        [Config("feelers.heading_ki")]
        public static double HeadingKi = 0.0;

        /// <summary>
        /// The derivative staticant for the heading PID
        /// </summary>
        [Config("feelers.heading_kd")]
        public static double HeadingKd = 0.0;
    }

    public static class WaypointSubsystem
    {
        // gps-related config
        /// <summary> 
        /// filename for the waypoints (should be CSV file with label,lat,lon,)
        /// </summary>
        public static string WaypointsFilename = "resources/waypoints.csv";

        /// <summary>
        /// How close we have to be for a GPS waypoint to be considered 'reached,' in meters
        /// </summary>
        [Config("waypoints.waypoint_pop_dist")]
        public static double WaypointPopDist = 1.5;

        /// <summary>
        /// How long we have to be within the WaypointPopDist, in milliseconds
        /// </summary>
        [Config("waypoints.waypoint_pop_time")]
        public static ulong WaypointPopTime = 500;

        /// <summary>
        /// How long to wait, after starting the run, before factoring in the GPS waypoints, in milliseconds
        /// </summary>
        [Config("waypoints.gps_wait_time")]
        public static ulong GpsWaitTime = 1000 * 2;

        /// <summary>
        /// Longitude of the west-most edge of the practice autonav field
        /// </summary>
        public static double PracticeLongitude = -83.218909;

        /// <summary>
        /// Longitude of the west-most edge of the autonav competition field
        /// </summary>
        public static double AutonavLongitude = -83.219584;

        /// <summary>
        /// Longitude of the west-most edge of the selfdrive course
        /// </summary>
        public static double SelfdriveLongitude = -83.217515;

        /// <summary>
        /// Latitude of the northern edge of the engineering quadrangle on OU campus.
        /// </summary>
        public static double EquadLatitude = 35.211160;

        /// <summary>
        /// Whether we are trying to do qualification or not
        /// </summary>
        public static bool Qualificaiton = false;
    }

    public static class FakeCameraSubsystemConfig
    {
        /// <summary>
        /// Filename of video to send as raw camera frames
        /// </summary>
        public static string Filename = "resources/video/camera.mp4";

        /// <summary>
        /// Frames to publish per second
        /// </summary>
        public static double FPS = 60; //FIXME I don't think this gets actually respected very well
    }

    public static class FakeGpsSubsystem
    {
        public static string Filename = "resources/gps/ENTRY_GPS.csv";
    }
}
