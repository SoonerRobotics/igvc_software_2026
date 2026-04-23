using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Core.Units;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Subsystems.Tools;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Core.Hardware;

namespace igvc_csharp.Subsystems.FeelerSubsystem;


/**
 * Configuration parameters for the feeler node
 * @param max_length is the maximum/default length for a feeler
 * @param number_of_feelers controls how many feelers there are, distributed uniformly in a circle
 * @param start_angle the starting angle offset when building the feelers, in degrees
 */
//FIXME I think this got superseded by Configuration.cs???
struct FeelerNodeConfig
{
    public int max_length; // pixels
    public int number_of_feelers;
    public double angle_offset; // degrees at which the first feeler starts, 0 degrees is the positive X-axis
    public double angular_width; // how many degrees from the angle_offset is the last feeler
    public bool balance_feelers; // whether to build backwards feelers or not (feelers behind/180-degrees offset from the initial feelers)
    //FIXME add a waypointPopWaitTime???
    public double waypointPopDist; // how close to a waypoint do we have to be, in meters, to consider it reached
    public ulong gpsWaitMilliseconds; // time to wait before using GPS waypoints, in milliseconds
    public int gpsBiasWeight; // pixels
    public int forwardBiasWeight; // pixels
    public int backwardsBiasWeight; // pixels
    public double max_turn_speed; // meters per second
    public double max_drive_speed; // meters per second
    public double max_strafe_speed; // meters per second
};



// Subsystem to do the actual reactive image feelering
// other actual path planning / motor / state management will be done by separate class FIXME TODO will it actually tho?
[Subsystem("FeelerSubsystem", Disabled = true, DependsOn = [typeof(ControllerSubsystem)])]

public class FeelerSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    // feelers
    private List<Feeler>? _feelers;
    private Feeler _headingArrow = new Feeler(0, 0, new Scalar(0, 0, 0)); //FIXME default color

    // PID controllers
    private Tools.PIDController _headingPID = new(0.0, 0.0, 0.0);

    // config
    FeelerNodeConfig _config;

    // FIXME these should be pointers or something???
    private readonly Channel<byte[]> _maskChannel = Channel.CreateBounded<byte[]>(1);
    private readonly Channel<byte[]> _debugChannel = Channel.CreateBounded<byte[]>(1);

    // GPS
    private Feeler _gpsFeeler = new Feeler(0, 0, new Scalar(0, 0, 0)); //FIXME fix default color
    private LatLng? _goalPoint;
    private VectorNavReport _position; //????? FIXME
    private double _distToWaypoint = 0;
    private ulong _lastTime = 0;
    private ulong _gpsTime = 0;

    // feedback
    private bool _hasPlayedGps = false;
    private bool _hasPlayedHorn = false;

    // stuff for file-reading code (copied and pasted from https://github.com/SoonerRobotics/autonav_software_2024/blob/feat/astar_rewrite_v3/autonav_ws/src/autonav_nav/src/astar.cpp)
    private readonly String _WAYPOINTS_FILENAME = "./data/waypoints.csv"; // filename for the waypoints (should be CSV file with label,lat,lon,)
    private Dictionary<String, List<LatLng>>? _waypointsDict; // dictionairy of lists containing the GPS waypoints we could PID to, choose the waypoints for the correct direction from here
    private int _waypointIndex = 0;
    private String _direction = ""; //FIXME make this an enum or something

    public override Task Init(CancellationToken token)
    {
        // configuration stuff
        var config = new FeelerNodeConfig
        {
            max_length = 100,
            number_of_feelers = 16,
            angle_offset = 2,
            angular_width = 180 - 4,
            balance_feelers = true,
            waypointPopDist = 2,
            gpsWaitMilliseconds = 5000 * 20,
            gpsBiasWeight = 0,
            forwardBiasWeight = 100,
            backwardsBiasWeight = 200,
            max_turn_speed = 1.25,
            max_drive_speed = 2.0,
            max_strafe_speed = 0.0,
        };

        _config = config;


        int numWaypoints = 0;
        // === read waypoints from file === (copied and pasted from last year's feat/astar_rewrite_v3 branch)
        String line;

        using (StreamReader waypointsFile = new(_WAYPOINTS_FILENAME))
        {
            // skip the first line
            line = waypointsFile.ReadLine();
            while ((line = waypointsFile.ReadLine()) != null)
            {
                var tokens = line.Split(",");

                LatLng point = new(
                    double.Parse(tokens[1]),
                    double.Parse(tokens[2])
                );

                // waypoints are stored like {"north":[GPSPoint, GPSPoint]}
                _waypointsDict[tokens[0]].Add(point);
                numWaypoints++;
            }
        }
        _waypointIndex = 0;
        // === /read waypoints ===

        // log("Number of waypoints read: " + numWaypoints, AutoNav::Logging::INFO);

        if (numWaypoints < 1)
        {
            // log("No waypoints read! GPS Feeler will not work", AutoNav::Logging::WARN);
        }

        // make all the feelers
        _feelers = [];
        BuildFeelers();

        _lastTime = TimeUtils.Now();

        // pid controllers
        _headingPID = new Tools.PIDController(.002, 0.0, 0.0);

        // subscribers
        SubscribeMessage<VectorNavReport>(MessageType.Gps, OnPositionReceived, token);
        SubscribeImage("front_view", OnImageReceived, token);
        SubscribeImage("front_view_debug", OnDebugImageReceived, token);

        // publishers
        _ = Task.Run(() => PublishOutputMessages(token), token);

        SetOperatingState(SubsystemState.Ready);

        _hasPlayedGps = false;
        _hasPlayedHorn = false;

        return Task.CompletedTask;
    }


    public override Task OnRobotModeChanged(RobotModeEnum old, RobotModeEnum current)
    {
        if (old == RobotModeEnum.Autonomous && current != RobotModeEnum.Autonomous)
        {
            canbus.SafetyLights.SetAutonomous();
        }
        else if (current == RobotModeEnum.Autonomous)
        {
            canbus.SafetyLights.SetAutonomous();
        }
        else if (current == RobotModeEnum.Manual)
        {
            canbus.SafetyLights.SetManual();
        }
        else
        {
            canbus.SafetyLights.SetDisabled(); //FIXME ???
        }

        return Task.CompletedTask;
    }

    public override void on_config_updated(json old_cfg, json new_cfg)
    {
        var new_config = new_cfg.get<FeelerNodeConfig>();
        _config = new_config;

        BuildFeelers();
    }

    /**
     * Builds the list of feelers based on configuration parameters.
     * Evenly distributes a num_feelers number of feelers in a circle of radius max_length
     * starting with an offset of start_angle (in degrees).
     */
    public void BuildFeelers()
    {
        // reset feelers
        _feelers ??= [];
        _feelers.Clear();

        var start_angle = new Angle(_config.angle_offset, false);
        var end_angle = new Angle(_config.angle_offset + _config.angular_width, false);
        var angle_increment = new Angle(_config.angular_width / _config.number_of_feelers, false);

        // default to blue FIXME we want lerp to be automatic right?
        var defaultColor = new Scalar(100, 100, 200);

        for (var angle = start_angle; angle < end_angle; angle += angle_increment)
        {
            int x = (int)(_config.max_length * Math.Cos(angle.To(AngleUnit.Radians)));
            int y = (int)(_config.max_length * Math.Sin(angle.To(AngleUnit.Radians)));

            _feelers.Add(new Feeler(x, y, defaultColor));
        }

        // build some feelers on the other side of the cone/arc formed from start_angle to end_angle
        if (_config.balance_feelers)
        {
            var flipped_start = (start_angle + new Angle(180, false)).WrapAngle();
            var flipped_end = (flipped_start + new Angle(_config.angular_width, false)).WrapAngle();

            for (var angle = flipped_start; angle < flipped_end; angle += angle_increment)
            {
                int x = (int)(_config.max_length * Math.Cos(angle.To(AngleUnit.Radians)));
                int y = (int)(_config.max_length * Math.Sin(angle.To(AngleUnit.Radians)));

                _feelers.Add(new Feeler(x, y, defaultColor));
            }
        }

        // bias feelers forwards
        var forwardsFeeler = new Feeler(0, 100, defaultColor); // positive y is upwards in an image
        foreach (var feeler in _feelers)
        {
            //FIXME this biases the revers feelers as well right??? shouldn't we NOT DO THIS???
            feeler.Bias(_config.forwardBiasWeight * (feeler * forwardsFeeler));
        }

        // bias backwards feelers backwards TODO FIXME this doesn't work
        // Feeler backwardsFeeler = Feeler(10, -50);
        // for (int i = 0; i < _feelers.Count(); i++) {
        //     _feelers.at(i).bias(_config.backwardsBiasWeight * (_feelers.at(i) * backwardsFeeler));

        //     // log("BIAS AMOUNT: " + std::to_string(_feelers.at(i).getBiasAmount()));
        // }

        // log("FEELERS BUILT! NUMBER OF FEELERS: " + std::to_string(_feelers.Count()), AutoNav::Logging::INFO);
    }

    /**
     * Callback for the combined image from the cameras.
     * @param image a compressedimage message with the combined transformations of all 4 cameras
     */
    private Task OnImageReceived(ImageFrame frame, CancellationToken token)
    {
        // once we've actually gotten an image, we can safely say we're operating pretty well
        if (State != SubsystemState.Operating)
        {
            SetOperatingState(SubsystemState.Operating);
        }

        // turn the image into a format we can use
        var mask = _maskChannel.Writer.TryWrite(frame.GetImageDataArray());

        // _perf_start("FeelerNode::update");

        // calculate new length of every new feeler
        foreach (var feeler in _feelers)
        {
            feeler.Update(mask);
        }

        // _perf_stop("FeelerNode::update", true);

        CalculateOutputs();

        return Task.CompletedTask;
    }

    /**
     * Callback to receive the position of the robot from the particle filter
     * Uses the position to calculate what direction we need to move in to get to next waypoint, as well as popping waypoints once we get close enough
     * Also automatically calculates if we're going north or south
     * All code that is related to position-based stuff is located here, as it doesn't get updated anywhere else
     * No output, but does updates GPS feeler, which is added to headingArrow later to drive us towards the waypoint
     * @param msg the Postion message from the particle filter
     */
    public void OnPositionReceived(autonav_msgs::msg::Position msg)
    {
        _position = msg;

        // if we haven't set a timestamp yet, but have started the run
        if (_gpsTime == 0 && Robot.Instance.State.MotionAllowed && Robot.Instance.State.Mode == RobotModeEnum.Autonomous)
        {
            _gpsTime = TimeUtils.Now(); // then set the timestamp for the start of the run
        }

        // if, however, we have set a timestamp, and it's been long enough that the particle filter should kTimeUtils.Now which direction we're heading
        else if (_gpsTime != 0 && (TimeUtils.Now() - _gpsTime > _config.gpsWaitMilliseconds) && _direction == "")
        {
            // then pick a set of waypoints based on which direction we are heading
            double heading_degrees = Math.Abs(_position.theta * 180 / Math.PI);
            if (120 < heading_degrees && heading_degrees < 240)
            {
                _direction = "compSouth";
                // log("PICKING SOUTH WAYPOINTS", AutoNav::Logging::INFO);
            }
            else
            {
                _direction = "compNorth";
                // log("PICKING NORTH WAYPOINTS", AutoNav::Logging::INFO);
            }
        }

        _distToWaypoint = 0;
        // if we have a direction, then we are good to use it to get waypoints and go towards them
        if (_direction != "")
        {
            // if we don't have any waypoints, however
            if (_waypointsDict.Count() == 0)
            {
                //TODO do something???
                return;
            }
            LatLng goalPoint = _waypointsDict[_direction][_waypointIndex];

            // make a vector pointing towards the GPS waypoint
            int latError = (goalPoint.Latitude - _position.latitude) * _latitudeLength * 5;
            int lonError = (goalPoint.Longitude - _position.longitude) * _longitudeLength * 5;
            double angleToWaypoint = std::atan2(latError, lonError); // all in radians, don't worry

            // account for rotation of the robot (aka translate the gps error into camera/robot-relative coordinates, where (0, 0) is the center of the camera frame)
            double headingError = (angleToWaypoint - _position.theta); //TODO FIXME double check this
            int gps_x = (int)((lonError * Math.Cos(headingError)) - (latError * Math.Sin(headingError)));
            int gps_y = (int)((lonError * Math.Sin(headingError)) + (latError * Math.Cos(headingError)));
            _gpsFeeler = new Feeler(gps_x, gps_y);

            // Feeler velocityFeeler = Feeler(_position.x_vel, _position.y_vel);
            Feeler velocityFeeler = new(0, 100);

            // calculate bias for every feeler
            for (int i = 0; i < _feelers.Count(); i++)
            {
                double gps_bias = _config.gpsBiasWeight * (_feelers[i] * _gpsFeeler); // dot product (normalized, don't worry)
                double forward_bias = _config.forwardBiasWeight * (_feelers[i] * velocityFeeler); // dot product

                if (gps_bias < 0.0)
                {
                    gps_bias = 0.0;
                }

                if (forward_bias < 0.0)
                {
                    forward_bias = 0.0;
                }

                _feelers[i].Bias(gps_bias + forward_bias);
            }

            _distToWaypoint = Math.Sqrt(Math.Pow((goalPoint.lon - _position.longitude) * _latitudeLength, 2) + Math.Pow((goalPoint.lat - _position.latitude) * _longitudeLength, 2));

            // if we are close enough to the waypoint, and we aren't going to cause an out-of-bounds index error
            if (_distToWaypoint < _config.waypointPopDist && _waypointIndex < (_waypointsDict[_direction].Count() - 2))
            {
                // then go to the next waypoint
                _waypointIndex++;

                autonav_msgs::msg::WaypointReached msg;
                msg.latitude = goalPoint.lat;
                msg.longitude = goalPoint.lon;
                msg.tag = "feelers";

                _waypointPublisher.publish(msg);

                // log("NEXT WAYPOINT!", AutoNav::Logging::WARN);
            }
        }

        CalculateOutputs();
    }

    /**
     * Callback to receive the color image to draw debug information on
     * All draw() calls should be in this function.
     * @param image the compressedImage message to draw the feelers on
     */
    private Task OnDebugImageReceived(sensor_msgs::msg::CompressedImage image)
    {
        // update the headingArrow with the most recent information
        CalculateOutputs();

        // log("GETTING DEBUG IMAGE!", AutoNav::Logging::WARN);

        // get the debug image
        _debugChannel.Writer.TryWrite(FrameSource.GetImageDataArray());

        // don't publish or draw on the image if it doesn't exist
        if (_debug_image_ptr == null)
        {
            return;
        }

        // draw feelers on the debug image
        // _perf_start("FeelerNode::draw");
        foreach (var feeler in _feelers)
        {
            // color biased feelers differently TODO FIXME why do we need to recalculate this every time?
            // TODO we should change Feeler.cs to pass in 2 colors and have it lerp automatically in draw() or something...
            Scalar color_ = Feeler.Lerp(BLUE, RED, feeler.GetBiasAmount() / (_config.forwardBiasWeight));
            feeler.SetColor(color_);

            feeler.Draw(_debug_image_ptr);
        }

        // draw feeler towards GPS waypoint
        _gpsFeeler.Draw(_debug_image_ptr);

        // draw the heading arrow on top of everything else
        _headingArrow.Draw(_debug_image_ptr);
        // _perf_stop("FeelerNode::draw", true);

        // publish the debug image

        var detections = _detector!.Detect(jpeg);
        var annotated = OpenCvDetectionRenderer.RenderDetections(jpeg, detections);
        var frame = MessageConstructor.CreateImageFrame(640, 480, "yolo_view", annotated);
        var wrappedFrame = MessageWrapper.From(MessageType.ImageFrame, frame.ByteBuffer.ToFullArray());
        EventBus.Instance.Publish(new MessageWrapperEvent(wrappedFrame));

        return Task.CompletedTask;
    }

    /**
     * Calculate what the motor output should be, based on all available sensor inputs.
     * Main feeler node function, called in every sensor callback, as new sensor data = new motor outputs.
     * Motor outputs (aka _headingArrow) calculated here will be read by publishOutputMessages() to be put into message form and sent,
     * as publishOutputMessages() runs much faster than all the sensor inputs for safety reasons, as the firmware
     * on the motor manager PCB will disable the motors if it hasn't received a motor command after a short period of time.
     */
    public void CalculateOutputs()
    {
        // reinitialize the heading arrow
        _headingArrow = new Feeler(0, 0);
        _headingArrow.SetColor(new Scalar(200, 200, 0));

        // add all the feelers together
        foreach (var feeler in _feelers)
        {
            //FIXME the weight of the feelers should be configurable (outside of MAX_LENGTH), or like give them a custom response curve or something
            _headingArrow = _headingArrow + feeler;
        }
    }

    /**
     * Publish all the output messages (motors, audible feedback, and safety lights).
     * On a short timer so we don't fail the firmware heartbeat watchdog timer check thingamajig.
     */
    public void PublishOutputMessages()
    {
        // if we aren't in autonomous
        if ((Robot.Instance.State.Mode != RobotModeEnum.Autonomous) || (State != SubsystemState.Operating))
        {
            return; // return because we don't need to do anything (so as to avoid conflicting with manual control if that's running)
        }

        // make the messages for publishing
        autonav_msgs::msg::MotorInput msg;
        autonav_msgs::msg::AudibleFeedback feedback_msg;

        // but if we ARE in autonomous,
        canbus.SafetyLights.SetAutonomous();

        // if we are allowed to move (earlier check means we are already in auto and operating, so don't have to recheck those)
        if (Robot.Instance.State.MotionAllowed)
        {
            // convert headingArrow to motor outputs
            //FIXME we want to be going max speed on the straightaways
            //FIXME the clamping should be configurable or something
            double multiplier = 1.0;
            if (!_config.balance_feelers)
            {
                multiplier = 5.0;
            }
            msg.forward_velocity = Math.Clamp(_headingArrow.GetY() * multiplier, -_config.max_drive_speed, _config.max_drive_speed); //FIXME configure divider number thingy
            msg.sideways_velocity = 0.0;
            msg.angular_velocity = Math.Clamp(_headingPID.Calculate(_headingArrow.GetX()), -_config.max_turn_speed, _config.max_turn_speed); // one camera for TimeUtils.Now so always turn, no strafe

            //TODO FIXME these are like, kinda jank hacks to get it to work, it should not be like this in the final version
            // if feelers doesn't produce any motor command (if it's in a symmetrical position)
            if (Math.Abs(msg.forward_velocity) < 0.1 && Math.Abs(msg.angular_velocity) < 0.1)
            {
                // then assume something is bad and go backwards and to the left
                msg.forward_velocity = -0.5;
                msg.angular_velocity = 0.2;

                // if we are going backwards and not really turning
            }
            else if (msg.forward_velocity < 0.0 && Math.Abs(msg.angular_velocity) < 0.1)
            {
                msg.angular_velocity *= 3; // then go faster
                msg.angular_velocity = Math.Clamp(msg.angular_velocity, -_config.max_turn_speed, _config.max_turn_speed);
            }

            //TODO safety lights need to change to other colors and stuff for debug information
        }
        else
        {
            // we are not mobility enabled and thus not allowed to move, so publish velocities of 0 for everything
            msg.forward_velocity = 0.0;
            msg.sideways_velocity = 0.0;
            msg.angular_velocity = 0.0;
        }

        //TODO figure out what sounds we actually want to play and when
        bool publishAudible = true;
        // if we reached a waypoint
        if (distToWaypoint < _config.waypointPopDist && _direction != "" && !_hasPlayedGps)
        {
            feedback_msg.filename = "~/autonav_software_2025/music/mine_xp.mp3";

            canbus.SafetyLights.FlashTemporary(ColorUtils.Color.Green, token, length: 2000);

            _hasPlayedGps = true;
        }
        // if we've found the direction ??? FIXME
        else if (_direction != "" && !_hasPlayedHorn)
        {
            feedback_msg.filename = "~/autonav_software_2025/music/windows-xp-startup.mp3";
            _hasPlayedHorn = true;
        }
        else
        {
            publishAudible = false;
        }

        // publish the messages
        _motorPublisher.publish(msg);

        // if we are actually wanting to play a file
        if (publishAudible)
        {
            _audibleFeedbackPublisher.publish(feedback_msg);
        }
    }
}