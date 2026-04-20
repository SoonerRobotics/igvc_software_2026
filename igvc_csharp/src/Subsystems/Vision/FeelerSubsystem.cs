

using System.Diagnostics;

namespace igvc_csharp.Subsystems.Feeler;


/**
 * Configuration parameters for the feeler node
 * @param max_length is the maximum/default length for a feeler
 * @param number_of_feelers controls how many feelers there are, distributed uniformly in a circle
 * @param start_angle the starting angle offset when building the feelers, in degrees
 */
struct FeelerNodeConfig
{
    int max_length; // pixels
    int number_of_feelers;
    double start_angle; // degrees
    double end_angle; // degrees
    bool balance_feelers; // TODO FIXME whether to make feelers like, symmetrical
    double waypointPopDist; // meters?
    double ultrasonic_contribution; // weight between 0 and 2 (or higher)
    unsigned long gpsWaitMilliseconds; // time to wait before using GPS waypoints, in milliseconds
    int gpsBiasWeight; // pixels
    int forwardBiasWeight; // pixels
    int backwardsBiasWeight; // pixels
    float max_turn_speed; // meters per second, probably (check sparkmax_node.py)
    float max_drive_speed; // meters per second
    float max_strafe_speed; // meters per second

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(FeelerNodeConfig, max_length, number_of_feelers, start_angle, end_angle, balance_feelers, waypointPopDist, ultrasonic_contribution, gpsWaitMilliseconds, gpsBiasWeight, forwardBiasWeight);
};



// Subsystem to do the actual reactive image feelering
// other actual path planning / motor / state management will be done by separate class
[Subsystem("FeelerSubsystem", Disabled = true)]
public class FeelerSubsystem : SubsystemBase
{
    public FeelerSubsystem()
    {
        // configuration stuff
        var config = FeelerNodeConfig();
        config.max_length = 100;
        config.number_of_feelers = 16;
        config.start_angle = 25;
        config.end_angle = 180 - config.start_angle;
        config.balance_feelers = true;
        config.waypointPopDist = 2;
        config.ultrasonic_contribution = 1;
        config.gpsWaitMilliseconds = 5000 * 20;
        config.gpsBiasWeight = 0;
        config.forwardBiasWeight = 100;
        config.backwardsBiasWeight = 200;
        config.max_turn_speed = 1.25;
        config.max_drive_speed = 2.0;
        config.max_strafe_speed = 0.0;

        __config = config;
        _config = config;
    }

    public override Task Init(CancellationToken token)
    {
        return Task.CompletedTask;
    }

    // === read waypoints from file === (copied and pasted from last year's feat/astar_rewrite_v3 branch)
    String line;
    _waypointsFile.open(_WAYPOINTS_FILENAME);
        getline(waypointsFile, line); // skip the first line
    int numWaypoints = 0;
        while (getline(waypointsFile, line)) {
            List<String> tokens; // https://www.geeksforgeeks.org/tokenizing-a-string-cpp/
    Stringstream strstream(line);
    String intermediate;
            while (getline(strstream, intermediate, ','))
            {
                tokens.push_back(intermediate);
            }

            GPSPoint point;
            point.lat = std::stod(tokens[1]); //https://cplusplus.com/reference/string/stod/
            point.lon = std::stod(tokens[2]);

            // waypoints are stored like {"north":[GPSPoint, GPSPoint]}
            waypointsDict[tokens[0]].push_back(point);
numWaypoints++;
        }
        waypointsFile.close();
_waypointIndex = 0;
// === /read waypoints ===

log("Number of waypoints read: " + std::to_string(numWaypoints), AutoNav::Logging::INFO);

if (numWaypoints < 1)
{
    log("No waypoints read! GPS Feeler will not work", AutoNav::Logging::WARN);
}

// make all the feelers
_buildFeelers();

// make the feelers for the ultrasonics
//TODO this should be part of buildFeelers or its own function or something
ultrasonic_feelers = List<Feeler>();
for (double angle = 0.0; angle < 360; angle += 90)
{ // these originate at the origin, which is fine because the only contribute in one axis
    int x = _config.max_length * cos(radians(angle)); //int should truncate these to nice whole numbers
    int y = _config.max_length * sin(radians(angle));

    _ultrasonic_feelers.push_back(Feeler(x, y));
    _ultrasonic_feelers.push_back(Feeler(x, y)); // there are 2 ultrasonic distance sensors per side
}

for (Feeler & feeler : ultrasonic_feelers)
{
    feeler.setColor(cv::Scalar(0, 200, 100)); // ultrasonic feelers are a different color
}

lastTime = now();

// pid controllers
_headingPID = PIDController(.002, 0.0, 0.0);

// subscribers
positionSubscriber = create_subscription<autonav_msgs::msg::Position>("/autonav/position", 1, std::bind(&FeelerNode::onPositionReceived, this, std::placeholders::_1));
imageSubscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/vision/combined/filtered", 1, std::bind(&FeelerNode::onImageReceived, this, std::placeholders::_1));
debugImageSubscriber = create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/vision/combined/debug", 1, std::bind(&FeelerNode::onDebugImageReceived, this, std::placeholders::_1));
ultrasonicSubscriber = create_subscription<autonav_msgs::msg::Ultrasonic>("/autonav/ultrasonics", 1, std::bind(&FeelerNode::onUltrasonicsReceived, this, std::placeholders::_1));

// publishers
motorPublisher = create_publisher<autonav_msgs::msg::MotorInput>("/autonav/motor_input", 1);
debugPublisher = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/feelers/debug", 1);
safetyLightsPublisher = create_publisher<autonav_msgs::msg::SafetyLights>("/autonav/safety_lights", 1);
audibleFeedbackPublisher = create_publisher<autonav_msgs::msg::AudibleFeedback>("/autonav/audible_feedback", 1);
waypointPublisher = _create_publisher<autonav_msgs::msg::WaypointReached>("/autonav/waypoint_reached", 1);
publishTimer = _create_wall_timer(std::chrono::milliseconds(50), std::bind(&FeelerNode::publishOutputMessages, this));

set_device_state(AutoNav::DeviceState::READY);

// log("FEELERS READY!", AutoNav::Logging::WARN); //FIXME TODO
//FIXME this is for temporary debug purposes while we are minus a UI
// _set_system_state(AutoNav::SystemState::AUTONOMOUS, true);

_hasPlayedGps = false;
_hasPlayedHorn = false;
// _old_state = AutoNav::DeviceState::READY;

_on_system_state_updated(AutoNav::SystemState::SHUTDOWN, AutoNav::SystemState::DISABLED);
    }

    public void on_system_state_updated(AutoNav::SystemState old, AutoNav::SystemState new_state) override {
    autonav_msgs::msg::SafetyLights msg;
    msg.brightness = 200;
    msg.blink_period = 0;
    msg.mode = 1;

        // _log("Safety lights: " + std::to_string(msg.brightness) + ", " + std::to_string(msg.blink_period) + ", " + std::to_string(msg.mode), AutoNav::Logging::INFO);

        if (old == AutoNav::SystemState::AUTONOMOUS && new_state != AutoNav::SystemState::AUTONOMOUS) {
        // rainbow
        msg.mode = 3;
        _safetyLightsPublisher->publish(msg);
    } else if (new_state == AutoNav::SystemState::AUTONOMOUS) {
        msg.blink_period = 30;
        msg.red = 250;
        msg.blue = 250;
        msg.green = 250;
        msg.mode = 2; // auto
        _safetyLightsPublisher->publish(msg);
    } else if (new_state == AutoNav::SystemState::MANUAL) {
        msg.red = 200;
        msg.green = 200;
        msg.blue = 0;
        msg.mode = 1;
        _safetyLightsPublisher->publish(msg);
    }

    // _old_state = new_state;
}

public void on_config_updated(json &old_cfg, json &new_cfg) override {
    auto new_config = new_cfg.get< FeelerNodeConfig > ();
    _config = new_config;

    _buildFeelers();
    }

    /**
     * Builds the list of feelers based on configuration parameters.
     * Evenly distributes a num_feelers number of feelers in a circle of radius max_length
     * starting with an offset of start_angle (in degrees).
     */
    public void BuildFeelers()
{
    _feelers = List<Feeler>();
    for (double angle = _config.start_angle; angle < _config.end_angle; angle += ((_config.end_angle - _config.start_angle) / _config.number_of_feelers))
    {
        int x = _config.max_length * cos(radians(angle)); //int should truncate these to nice whole numbers
        int y = _config.max_length * sin(radians(angle));

        _feelers.push_back(Feeler(x, y));
    }

    // build some feelers on the other side of the cone/arc formed from start_angle to end_angle
    if (_config.balance_feelers)
    {
        for (double angle = wrapAngle(_config.start_angle + 180); angle < wrapAngle(_config.end_angle + 180); angle += ((_config.end_angle - _config.start_angle) / _config.number_of_feelers))
        {
            int x = _config.max_length * cos(radians(angle)); //int should truncate these to nice whole numbers
            int y = _config.max_length * sin(radians(angle));

            _feelers.push_back(Feeler(x, y));
        }
    }

    // bias feelers forwards
    Feeler forwardsFeeler = Feeler(10, 100); // positive (?!) y is upwards in an image
    for (int i = 0; i < _feelers.size(); i++)
    {
        _feelers.at(i).bias(_config.forwardBiasWeight * (_feelers.at(i) * forwardsFeeler));

        // log("BIAS AMOUNT: " + std::to_string(_feelers.at(i).getBiasAmount()));
    }

    // bias backwards feelers backwards TODO FIXME this doesn't work
    // Feeler backwardsFeeler = Feeler(10, -50);
    // for (int i = 0; i < _feelers.size(); i++) {
    //     _feelers.at(i).bias(_config.backwardsBiasWeight * (_feelers.at(i) * backwardsFeeler));

    //     // log("BIAS AMOUNT: " + std::to_string(_feelers.at(i).getBiasAmount()));
    // }

    log("FEELERS BUILT! NUMBER OF FEELERS: " + std::to_string(_feelers.size()), AutoNav::Logging::INFO);
}

/**
 * Callback for the combined image from the cameras.
 * @param image a compressedimage message with the combined transformations of all 4 cameras
 */
public void onImageReceived(sensor_msgs::msg::CompressedImage image)
{
    // log("FEELERS OPERATING!", AutoNav::Logging::WARN); //FIXME TODO

    // once we've actually gotten an image, we can safely say we're operating pretty well
    if (_get_device_state() != AutoNav::DeviceState::OPERATING)
    {
        set_device_state(AutoNav::DeviceState::OPERATING);
    }

    // turn the image into a format we can use
    auto mask = cv_bridge::toCvCopy(image)->image; //TODO what encoding do we want to use?
                                                   // _feeler_img_ptr = cv_bridge::toCvCopy(image);

    // log("LOGGING REGULAR IMAGE...", AutoNav::Logging::ERROR); //FIXME TODO

    //TODO TEMP HACK FIXME BUG draw on image to see if it has any effect
    // Feeler tempFeeler = Feeler(100, 100);
    // tempFeeler.setColor(cv::Scalar(200, 200, 100));
    // tempFeeler.draw(mask);

    // log("drew that one image", AutoNav::Logging::INFO); //FIXME TODO

    // _perf_start("FeelerNode::update");

    // calculate new length of every new feeler
    for (Feeler & feeler : _feelers)
        {
            feeler.update(&mask, this);
            // log(feeler.to_string(), AutoNav::Logging::WARN); //FIXME TODO
        }

        // log("first feeler: " + _feelers.at(0).to_string(), AutoNav::Logging::INFO);

        // _perf_stop("FeelerNode::update", true);

        // log("FEELERS DRAWING!", AutoNav::Logging::WARN); //FIXME TODO
        // log("FEELERS LENGTH, MASK ROWS, MASK COLS, DEBUG ROWS, DEBUG COLS", AutoNav::Logging::WARN);
        // log(std::to_string(_feelers.size()), AutoNav::Logging::WARN);
        // log(std::to_string(mask.cols), AutoNav::Logging::WARN);
        // log(std::to_string(mask.rows), AutoNav::Logging::WARN);
        // log(std::to_string(debug_image_ptr->cols), AutoNav::Logging::WARN);
        // log(std::to_string(debug_image_ptr->rows), AutoNav::Logging::WARN);

        // log("chat are we cooked", AutoNav::Logging::WARN); //FIXME TODO

        _calculateOutputs();
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

    // log("GOT GPS!", AutoNav::Logging::INFO);

    // if we haven't set a timestamp yet, but have started the run
    if (_gpsTime == 0 && _is_mobility() && _get_system_state() == AutoNav::SystemState::AUTONOMOUS)
    {
        _gpsTime = now(); // then set the timestamp for the start of the run
                          // if, however, we have set a timestamp, and it's been long enough that the particle filter should know which direction we're heading
    }
    else if (_gpsTime != 0 && (now() - _gpsTime > _config.gpsWaitMilliseconds) && _direction == "")
    {
        // then pick a set of waypoints based on which direction we are heading
        double heading_degrees = abs(_position.theta * 180 / PI);
        if (120 < heading_degrees && heading_degrees < 240)
        {
            _direction = "compSouth";
            log("PICKING SOUTH WAYPOINTS", AutoNav::Logging::INFO);
        }
        else
        {
            _direction = "compNorth";
            log("PICKING NORTH WAYPOINTS", AutoNav::Logging::INFO);
        }
    }

    _distToWaypoint = 0;
    if (_direction != "")
    { // if we have a direction, then we are good to use it to get waypoints and go towards them
      // if we don't have any waypoints, however
        if (_waypointsDict.size() == 0)
        {
            //TODO do something???
            return;
        }
        GPSPoint goalPoint = _waypointsDict.at(_direction)[_waypointIndex];

        // log("biasing my robot rn", AutoNav::Logging::INFO);

        // make a vector pointing towards the GPS waypoint
        int latError = (goalPoint.lat - _position.latitude) * _latitudeLength * 5;
        int lonError = (goalPoint.lon - _position.longitude) * _longitudeLength * 5;
        double angleToWaypoint = std::atan2(latError, lonError); // all in radians, don't worry

        // account for rotation of the robot (aka translate the gps error into camera/robot-relative coordinates, where (0, 0) is the center of the camera frame)
        double headingError = (angleToWaypoint - _position.theta); //TODO FIXME double check this
        int gps_x = (lonError * std::cos(headingError)) - (latError * std::sin(headingError));
        int gps_y = (lonError * std::sin(headingError)) + (latError * std::cos(headingError));
        _gpsFeeler = Feeler(gps_x, gps_y);

        // log("GPS FEELER: " + _gpsFeeler.to_string(), AutoNav::Logging::INFO);
        // log("ERROR: (" + std::to_string(lonError) + ", " + std::to_string(latError) + ")", AutoNav::Logging::INFO);
        // log("Heading: " + std::to_string(_position.theta));

        // Feeler velocityFeeler = Feeler(_position.x_vel, _position.y_vel);
        Feeler velocityFeeler = Feeler(0, 100);

        // calculate bias for every feeler
        for (int i = 0; i < _feelers.size(); i++)
        {
            double gps_bias = _config.gpsBiasWeight * (_feelers.at(i) * _gpsFeeler); // dot product (normalized, don't worry)
            double forward_bias = _config.forwardBiasWeight * (_feelers.at(i) * velocityFeeler); // dot product

            if (gps_bias < 0.0)
            {
                gps_bias = 0.0;
            }

            if (forward_bias < 0.0)
            {
                forward_bias = 0.0;
            }

            _feelers.at(i).bias(gps_bias + forward_bias);
        }

        // log("ROBOT has been BIASED", AutoNav::Logging::INFO);

        _distToWaypoint = std::sqrt(std::pow((goalPoint.lon - _position.longitude) * _latitudeLength, 2) + std::pow((goalPoint.lat - _position.latitude) * _longitudeLength, 2));

        // if we are close enough to the waypoint, and we aren't going to cause an out-of-bounds index error
        if (_distToWaypoint < config.waypointPopDist && _waypointIndex < (_waypointsDict[_direction].size() - 2))
        {
            // then go to the next waypoint
            _waypointIndex++;

            autonav_msgs::msg::WaypointReached msg;
            msg.latitude = goalPoint.lat;
            msg.longitude = goalPoint.lon;
            msg.tag = "feelers";

            _waypointPublisher->publish(msg);

            log("NEXT WAYPOINT!", AutoNav::Logging::WARN);
        }
    }

    _calculateOutputs();
}

/**
 * Callback to get data from the ultrasonic sensors. This message does not contain the data for every sensor,
 * but rather data for one of the sensors (each sensor will get a message sent for it). For more information,
 * see the SCR 2025 AutoNav CAN specification
 * @param msg an Ultrasonic message from a sensor
 */
public void onUltrasonicsReceived(autonav_msgs::msg::Ultrasonic msg)
{
    // log("GETTING ULTRASONICS!", AutoNav::Logging::WARN); //FIXME TODO

    _ultrasonic_feelers[msg.id - 1].setLength(msg.distance * _config.ultrasonic_contribution); // minus 1 because the sensors are numbered 1-8

    // log("ULTRASONICS GOT!", AutoNav::Logging::WARN); //FIXME TODO

    _calculateOutputs();
}

/**
 * Callback to receive the color image to draw debug information on
 * All draw() calls should be in this function.
 * @param image the compressedImage message to draw the feelers on
 */
public void onDebugImageReceived(sensor_msgs::msg::CompressedImage image)
{
    // update the headingArrow with the most recent information
    _calculateOutputs();

    // log("GETTING DEBUG IMAGE!", AutoNav::Logging::WARN); //FIXME TODO

    // get the debug image
    _debug_image_ptr = cv_bridge::toCvCopy(image); //TODO figure out what encoding we want to use

    // don't publish or draw on the image if it doesn't exist
    if (_debug_image_ptr == nullptr)
    {
        return;
    }

    // log("LOGGING DEBUG IMAGE RECEIVED...", AutoNav::Logging::ERROR); //FIXME TODO

    // draw feelers on the debug image
    // _perf_start("FeelerNode::draw");
    for (Feeler & feeler : _feelers)
        {
            // log(feeler.to_string(), AutoNav::Logging::WARN);
            // log("==================", AutoNav::Logging::WARN);
            // feeler.setXY(100, 100);

            // color biased feelers differently TODO FIXME why do we need to recalculate this every time?
            cv::Scalar color_ = lerp(BLUE, RED, feeler.getBiasAmount() / (_config.forwardBiasWeight));
feeler.setColor(color_);

feeler.draw(_debug_image_ptr->image);
            // log(feeler.to_string(), AutoNav::Logging::WARN);
        }

        // log("DREW FEELERS!", AutoNav::Logging::ERROR); //FIXME TODO

        // draw the ultrasonic feelers on the image (on top of the vision feelers)
        // for (Feeler feeler : _ultrasonic_feelers) {
        //     feeler.draw(_debug_image_ptr->image);
        // }

        // draw feeler towards GPS waypoint
        _gpsFeeler.setColor(cv::Scalar(50, 200, 50));
_gpsFeeler.draw(_debug_image_ptr->image);

// log("GPS FEELER: " + _gpsFeeler.to_string(), AutoNav::Logging::INFO); // FIXME TODO

// draw the heading arrow on top of everything else
_headingArrow.draw(_debug_image_ptr->image);
// _perf_stop("FeelerNode::draw", true);

// publish the debug image
_debugPublisher->publish(*(debug_image_ptr->toCompressedImageMsg()));
    }

    /**
     * Calculate what the motor output should be, based on all available sensor inputs.
     * Main feeler node function, called in every sensor callback, as new sensor data = new motor outputs.
     * Motor outputs (aka _headingArrow) calculated here will be read by publishOutputMessages() to be put into message form and sent,
     * as publishOutputMessages() runs much faster than all the sensor inputs for safety reasons, as the firmware
     * on the motor manager PCB will disable the motors if it hasn't received a motor command after a short period of time.
     */
    public void calculateOutputs()
{
    // reinitialize the heading arrow
    _headingArrow = Feeler(0, 0);
    _headingArrow.setColor(cv::Scalar(200, 200, 0));

    // add all the feelers together
    for (Feeler feeler : _feelers)
        {
            //FIXME the weight of the feelers should be configurable (outside of MAX_LENGTH), or like give them a custom response curve or something
            _headingArrow = _headingArrow + feeler;
        }

        // add all the ultrasonic feelers together
        for (Feeler feeler : _ultrasonic_feelers)
{
    // FIXME TODO use the config ultrasonics_weight to determine how much ultrasonics effect the heading arrow
    // _headingArrow = _headingArrow + feeler;
}
    }

    /**
     * Publish all the output messages (motors, audible feedback, and safety lights).
     * On a short timer so we don't fail the firmware heartbeat watchdog timer check thingamajig.
     */
    public void publishOutputMessages()
{
    // if we aren't in autonomous
    if ((_get_system_state() != AutoNav::SystemState::AUTONOMOUS) || (_get_device_state() != AutoNav::DeviceState::OPERATING))
    {
        return; // return because we don't need to do anything (so as to apublic void conflicting with manual control if that's running)
    }

    // log("PUBLISHING MOTOR OUTPUT!", AutoNav::Logging::WARN); //FIXME TODO

    // make the messages for publishing
    autonav_msgs::msg::SafetyLights safetyLightsMsg;
    autonav_msgs::msg::MotorInput msg;
    autonav_msgs::msg::AudibleFeedback feedback_msg;

    // default in auto should be red
    safetyLightsMsg.red = 250;
    safetyLightsMsg.blue = 250;
    safetyLightsMsg.green = 250;
    safetyLightsMsg.brightness = 200;
    safetyLightsMsg.mode = 1; // if we passed the system state check at the beginning of the function and reach this line of code then we're in auto
    safetyLightsMsg.blink_period = 20;

    // if we are allowed to move (earlier check means we are already in auto and operating, so don't have to recheck those)
    if (_is_mobility())
    {
        // log("WE ARE MOBILE!", AutoNav::Logging::WARN); //FIXME TODO

        // convert headingArrow to motor outputs
        //FIXME we want to be going max speed on the straightaways
        //FIXME the clamping should be configurable or something
        float multiplier = 1.0;
        if (!_config.balance_feelers)
        {
            multiplier = 5.0;
        }
        msg.forward_velocity = std::clamp(static_cast<float>(_headingArrow.getY()) * multiplier, -_config.max_drive_speed, _config.max_drive_speed); //FIXME configure divider number thingy
        msg.sideways_velocity = 0.0;
        msg.angular_velocity = std::clamp(static_cast<float>(_headingPID.calculate(_headingArrow.getX())), -_config.max_turn_speed, _config.max_turn_speed); // one camera for now so always turn, no strafe

        //TODO FIXME these are like, kinda jank hacks to get it to work, it should not be like this in the final version
        // if feelers doesn't produce any motor command (if it's in a symmetrical position)
        if (abs(msg.forward_velocity) < 0.1 && abs(msg.angular_velocity) < 0.1)
        {
            // then assume something is bad and go backwards and to the left
            msg.forward_velocity = -0.5;
            msg.angular_velocity = 0.2;

            // if we are going backwards and not really turning
        }
        else if (msg.forward_velocity < 0.0 && abs(msg.angular_velocity) < 0.1)
        {
            msg.angular_velocity *= 3; // then go faster
            msg.angular_velocity = std::clamp(msg.angular_velocity, -_config.max_turn_speed, _config.max_turn_speed);
        }

        //TODO safety lights need to change to other colors and stuff for debug information
    }
    else
    {
        // log("NO MOBILITY!", AutoNav::Logging::WARN); //FIXME TODO

        // we are not mobility enabled and thus not allowed to move, so publish velocities of 0 for everything
        msg.forward_velocity = 0.0;
        msg.sideways_velocity = 0.0;
        msg.angular_velocity = 0.0;
    }

    //TODO figure out what sounds we actually want to play and when
    bool publishAudible = true;
    if (distToWaypoint < config.waypointPopDist && _direction != "" && !_hasPlayedGps)
    {
        feedback_msg.filename = "~/autonav_software_2025/music/mine_xp.mp3";

        // green for a little bit
        safetyLightsMsg.red = 10;
        safetyLightsMsg.blue = 10;
        safetyLightsMsg.green = 240;

        _hasPlayedGps = true;
    }
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
    _motorPublisher->publish(msg);
    _safetyLightsPublisher->publish(safetyLightsMsg);
    // log("MOTOR AND SAFETY LIGHTS PUBLISHED!", AutoNav::Logging::WARN); //FIXME TODO

    // if we are actually wanting to play a file
    if (publishAudible)
    {
        _audibleFeedbackPublisher->publish(feedback_msg);
        // log("PUBLISHING AUDIBLE FEEDBACK!", AutoNav::Logging::WARN); //FIXME TODO
    }
}

private:
    // feelers
    List<Feeler> feelers;
List<Feeler> ultrasonic_feelers;
Feeler headingArrow = Feeler(0, 0);

// PID controllers
PIDController headingPID = PIDController(0.0, 0.0, 0.0);

// config
FeelerNodeConfig config;

cv_bridge::CvImagePtr debug_image_ptr;
cv_bridge::CvImagePtr feeler_img_ptr;

// GPS
Feeler gpsFeeler = Feeler(0, 0);
GPSPoint goalPoint;
autonav_msgs::msg::Position position;
double distToWaypoint = 0;
unsigned long int lastTime = 0;
unsigned long int gpsTime = 0;

// feedback
bool hasPlayedGps = false;
bool hasPlayedHorn = false;

// subscribers
rclcpp::Subscription<autonav_msgs::msg::Position>::SharedPtr positionSubscriber;
rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr imageSubscriber;
rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr debugImageSubscriber;
rclcpp::Subscription<autonav_msgs::msg::Ultrasonic>::SharedPtr ultrasonicSubscriber;

// publishers
rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motorPublisher;
rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debugPublisher;
rclcpp::Publisher<autonav_msgs::msg::SafetyLights>::SharedPtr safetyLightsPublisher;
rclcpp::Publisher<autonav_msgs::msg::AudibleFeedback>::SharedPtr audibleFeedbackPublisher;
rclcpp::Publisher<autonav_msgs::msg::WaypointReached>::SharedPtr waypointPublisher;

rclcpp::TimerBase::SharedPtr publishTimer;

// stuff for file-reading code (copied and pasted from https://github.com/SoonerRobotics/autonav_software_2024/blob/feat/astar_rewrite_v3/autonav_ws/src/autonav_nav/src/astar.cpp)
String WAYPOINTS_FILENAME = "./data/waypoints.csv"; // filename for the waypoints (should be CSV file with label,lat,lon,)
std::ifstream waypointsFile; // actual C++ file object
std::unordered_map<String, List<GPSPoint>> waypointsDict; // dictionairy of lists containing the GPS waypoints we could PID to, choose the waypoints for the correct direction from here
int waypointIndex = 0;
String direction = "";
double latitudeLength = 110944.21; // copied/pasted from last year's code, might need to change based on simulator
double longitudeLength = 81978.2;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<FeelerNode> feeler_node = std::make_shared<FeelerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(feeler_node);
    executor.spin();
    executor.remove_node(feeler_node);
    rclcpp::shutdown();
    return 0;
}
}
