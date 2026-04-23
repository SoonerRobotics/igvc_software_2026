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

namespace igvc_csharp.Subsystems.FeelerSubsystem;


/**
 * Configuration parameters for the feeler node
 * @param max_length is the maximum/default length for a feeler
 * @param number_of_feelers controls how many feelers there are, distributed uniformly in a circle
 * @param start_angle the starting angle offset when building the feelers, in degrees
 */
struct FeelerNodeConfig
{
    public int max_length; // pixels
    public int number_of_feelers;
    public double start_angle; // degrees
    public double end_angle; // degrees
    public bool balance_feelers; // TODO FIXME whether to make feelers like, symmetrical
    public double waypointPopDist; // meters?
    public ulong gpsWaitMilliseconds; // time to wait before using GPS waypoints, in milliseconds
    public int gpsBiasWeight; // pixels
    public int forwardBiasWeight; // pixels
    public int backwardsBiasWeight; // pixels
    public double max_turn_speed; // meters per second, probably (check sparkmax_node.py)
    public double max_drive_speed; // meters per second
    public double max_strafe_speed; // meters per second

    // NLOHMANN_DEFINE_TYPE_INTRUSIVE(FeelerNodeConfig, max_length, number_of_feelers, start_angle, end_angle, balance_feelers, waypointPopDist, gpsWaitMilliseconds, gpsBiasWeight, forwardBiasWeight);
};



// Subsystem to do the actual reactive image feelering
// other actual path planning / motor / state management will be done by separate class FIXME TODO will it actually tho?
[Subsystem("FeelerSubsystem", Disabled = true, DependsOn=[typeof(ControllerSubsystem)])]

public class FeelerSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    // feelers
    private List<Feeler> _feelers;
    private Feeler _headingArrow = new Feeler(0, 0, new Scalar(0, 0, 0)); //FIXME default color

    // PID controllers
    private Tools.PIDController _headingPID = new(0.0, 0.0, 0.0);

    // config
    FeelerNodeConfig _config;

    // FIXME these should be pointers or something???
    private OpenCvSharp.Mat _debug_image_ptr;
    private OpenCvSharp.Mat _feeler_img_ptr;

    // GPS
    private Feeler _gpsFeeler = new Feeler(0, 0, new Scalar(0, 0, 0)); //FIXME fix default color
    private LatLng _goalPoint;
    private autonav_msgs::msg::Position _position;
    private double _distToWaypoint = 0;
    private ulong _lastTime = 0;
    private ulong _gpsTime = 0;

    // feedback
    private bool _hasPlayedGps = false;
    private bool _hasPlayedHorn = false;

    // subscribers
    //FIXME positionSubscriber;
    //FIXME imageSubscriber;
    //FIXME debugImageSubscriber;

    // publishers FIXME
    // motorPublisher;
    // debugPublisher;
    // audibleFeedbackPublisher;
    // waypointPublisher;
    // rclcpp::TimerBase::SharedPtr publishTimer;

    // stuff for file-reading code (copied and pasted from https://github.com/SoonerRobotics/autonav_software_2024/blob/feat/astar_rewrite_v3/autonav_ws/src/autonav_nav/src/astar.cpp)
    private readonly String _WAYPOINTS_FILENAME = "./data/waypoints.csv"; // filename for the waypoints (should be CSV file with label,lat,lon,)
    private Dictionary<String, List<LatLng>> _waypointsDict; // dictionairy of lists containing the GPS waypoints we could PID to, choose the waypoints for the correct direction from here
    private int _waypointIndex = 0;
    private String _direction = ""; //FIXME make this an enum or something

    public FeelerSubsystem()
    {
        // configuration stuff
        var config = new FeelerNodeConfig
        {
            max_length = 100,
            number_of_feelers = 16,
            start_angle = 2,
            end_angle = 180 - 2, //FIXME make this automatically always 180 out of phase?
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

        // __config = config;
        _config = config;
    }

    public override Task Init(CancellationToken token)
    {
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
        BuildFeelers();

        _lastTime = TimeUtils.Now();

        // pid controllers
        _headingPID = new Tools.PIDController(.002, 0.0, 0.0);

        // subscribers
        positionSubscriber = create_subscription<autonav_msgs::msg::Position>("/autonav/position", 1, std::bind(&FeelerNode::onPositionReceived, this, std::placeholders::_1));
        imageSubscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/vision/combined/filtered", 1, std::bind(&FeelerNode::onImageReceived, this, std::placeholders::_1));
        debugImageSubscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/vision/combined/debug", 1, std::bind(&FeelerNode::onDebugImageReceived, this, std::placeholders::_1));

        // publishers
        motorPublisher = create_publisher<autonav_msgs::msg::MotorInput>("/autonav/motor_input", 1);
        debugPublisher = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/feelers/debug", 1);
        audibleFeedbackPublisher = create_publisher<autonav_msgs::msg::AudibleFeedback>("/autonav/audible_feedback", 1);
        waypointPublisher = _create_publisher<autonav_msgs::msg::WaypointReached>("/autonav/waypoint_reached", 1);
        publishTimer = _create_wall_timer(std::chrono::milliseconds(50), std::bind(&FeelerNode::publishOutputMessages, this));

        set_device_state(AutoNav::DeviceState::READY);

        _hasPlayedGps = false;
        _hasPlayedHorn = false;
        // _old_state = AutoNav::DeviceState::READY;

        //FIXME SHUTDOWN doesn't exist... maybe this should be an on_subsystem_state_update or something?
        OnSystemStateUpdated(RobotModeEnum.Disabled, RobotModeEnum.Disabled);

        return Task.CompletedTask;
    }


    //FIXME shouldn't this get handled by like... Robot.cs or something?
    public override void OnSystemStateUpdated(RobotModeEnum old, RobotModeEnum new_state)
    {
        if (old == RobotModeEnum.Autonomous && new_state != RobotModeEnum.Autonomous)
        {
            // rainbow
            msg.mode = 3;
            _safetyLightsPublisher.publish(msg);
        }
        else if (new_state == RobotModeEnum.Autonomous)
        {
            msg.blink_period = 30;
            msg.red = 250;
            msg.blue = 250;
            msg.green = 250;
            msg.mode = 2; // auto
            _safetyLightsPublisher.publish(msg);
        }
        else if (new_state == RobotModeEnum.Manual)
        {
            msg.red = 200;
            msg.green = 200;
            msg.blue = 0;
            msg.mode = 1;
            _safetyLightsPublisher.publish(msg);
        }
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
        _feelers = [];
        for (double angle = _config.start_angle; angle < _config.end_angle; angle += ((_config.end_angle - _config.start_angle) / _config.number_of_feelers))
        {
            int x = _config.max_length * Math.Cos(radians(angle)); //int should truncate these to nice whole numbers
            int y = _config.max_length * Math.Sin(radians(angle));

            _feelers.Add(new Feeler(x, y, new Scalar(0, 0, 0))); //FIXME default color
        }

        // build some feelers on the other side of the cone/arc formed from start_angle to end_angle
        if (_config.balance_feelers)
        {
            for (double angle = wrapAngle(_config.start_angle + 180); angle < wrapAngle(_config.end_angle + 180); angle += ((_config.end_angle - _config.start_angle) / _config.number_of_feelers))
            {
                int x = _config.max_length * Math.Cos(Radians(angle)); //int should truncate these to nice whole numbers
                int y = _config.max_length * Math.Sin(Radians(angle));

                _feelers.Add(new Feeler(x, y, new Scalar(0, 0, 0))); //FIXME default color
            }
        }

        // bias feelers forwards
        var forwardsFeeler = new Feeler(10, 100, new Scalar(0, 0, 0)); // positive (?!) y is upwards in an image
        for (int i = 0; i < _feelers.Count(); i++)
        {
            _feelers[i].Bias(_config.forwardBiasWeight * (_feelers[i] * forwardsFeeler));
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
    public void OnImageReceived(sensor_msgs::msg::CompressedImage image)
    {
        // once we've actually gotten an image, we can safely say we're operating pretty well
        if (_get_device_state() != AutoNav::DeviceState::OPERATING)
        {
            set_device_state(AutoNav::DeviceState::OPERATING);
        }

        // turn the image into a format we can use
        var mask = cv_bridge::toCvCopy(image).image; //TODO what encoding do we want to use?

        // _feeler_img_ptr = cv_bridge::toCvCopy(image);
        // _perf_start("FeelerNode::update");

        // calculate new length of every new feeler
        foreach (var feeler in _feelers)
        {
            feeler.Update(mask);
        }

        // _perf_stop("FeelerNode::update", true);

        CalculateOutputs();
    }

    /**
     * Callback to receive the position of the robot from the particle filter
     * Uses the position to calculate what direction we need to move in to get to next waypoint, as well as popping waypoints once we get close enough
     * Also automatically calculates if we're going north or south
     * All code that is related to position-based stuff is located here, as it doesn't get updated anywhere else
     * No output, but does updates GPS feeler, which is added to headingArrow later to drive us towards the waypoint
     * @param msg the Postion message from the particle filter
     */
    public void onPositionReceived(autonav_msgs::msg::Position msg)
    {
        _position = msg;

        // if we haven't set a timestamp yet, but have started the run
        if (_gpsTime == 0 && _is_mobility() && _get_system_state() == RobotModeEnum.Autonomous)
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
    public void OnDebugImageReceived(sensor_msgs::msg::CompressedImage image)
    {
        // update the headingArrow with the most recent information
        CalculateOutputs();

        // log("GETTING DEBUG IMAGE!", AutoNav::Logging::WARN); //FIXME TODO

        // get the debug image
        _debug_image_ptr = cv_bridge::toCvCopy(image); //TODO figure out what encoding we want to use

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
            Scalar color_ = Feeler.Lerp(BLUE, RED, feeler.getBiasAmount() / (_config.forwardBiasWeight));
            feeler.setColor(color_);

            feeler.Draw(_debug_image_ptr);
        }

        // draw feeler towards GPS waypoint
        _gpsFeeler.SetColor(new Scalar(50, 200, 50));
        _gpsFeeler.Draw(_debug_image_ptr);

        // draw the heading arrow on top of everything else
        _headingArrow.Draw(_debug_image_ptr);
        // _perf_stop("FeelerNode::draw", true);

        // publish the debug image
        _debugPublisher.publish(*(debug_image_ptr.toCompressedImageMsg()));
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
        if ((_get_system_state() != RobotModeEnum.Autonomous) || (_get_device_state() != AutoNav::DeviceState::OPERATING))
        {
            return; // return because we don't need to do anything (so as to apublic void conflicting with manual control if that's running)
        }

        // make the messages for publishing
        autonav_msgs::msg::MotorInput msg;
        autonav_msgs::msg::AudibleFeedback feedback_msg;

        // but if we ARE in autonomous,
        canbus.SafetyLights.SetAutonomous();

        // if we are allowed to move (earlier check means we are already in auto and operating, so don't have to recheck those)
        if (_is_mobility())
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