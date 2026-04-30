using System.Diagnostics;
using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using igvc_csharp.Core.Units;
using igvc_csharp.Subsystems.Tools;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Core.Hardware;
using OpenCvSharp;
using igvc_csharp.src.Subsystems.Feelers;
using FeelerConfig = igvc_csharp.Configuration.FeelerSubsystem;


namespace igvc_csharp.scr.Subsystems.Feelers;

[Subsystem("FeelerSubsystem", Disabled = false)]
public class FeelerSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    // actual feeler stuff
    private List<Feeler> _feelers = [];
    private Feeler _headingArrow = new();
    private Feeler _gpsFeeler = new();

    // pid controllers
    private PIDController _drivingPID = new(
        Configuration.FeelerSubsystem.DriveKp,
        Configuration.FeelerSubsystem.DriveKi,
        Configuration.FeelerSubsystem.DriveKd
    );
    //FIXME we actually could want to like, no-rotate and only strafe, so we'd need to headingPID towards waypoints but also strafePID to avoid obstacles as well
    private PIDController _headingPID = new(
        Configuration.FeelerSubsystem.HeadingKp,
        Configuration.FeelerSubsystem.HeadingKi,
        Configuration.FeelerSubsystem.HeadingKd
    );

    // GPS stuff
    private VectorNavReport _position;
    private ulong _runStartTime = 0;
    private ulong _waypointTimeStart = 0;
    private LatLng? _startGpsPos;
    private LatLng? _goalPoint;
    private Dictionary<String, List<LatLng>> _waypointsDict = [];
    private int _waypointIndex = 0;
    private string _direction = ""; //FIXME make this an enum or something

    // OpenCV stuff
    private readonly Channel<ImageFrame> _debugFrameChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    private readonly Channel<ImageFrame> _maskFrameChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    // public FeelerSubsystem()
    // {
    //     //TODO FIXME?
    // }

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        ReadWaypointsFile();

        BuildFeelers();

        // subscribers
        SubscribeImage(
            "front_view",
            OnDebugImageReceived,
            token
        );

        SubscribeImage(
            "front_transformed_view",
            OnMaskReceived,
            token
        );

        SubscribeMessage<VectorNavReport>(
            MessageType.Gps,
            OnPositionReceived,
            token
        );

        // actual feelers => motor output function
        _ = Task.Factory.StartNew(
            () => PerformFeelers(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        SetOperatingState(SubsystemState.Ready);
        return Task.CompletedTask;
    }

    private Task ReadWaypointsFile()
    {
        int numWaypoints = 0;
        // === read waypoints from file === (copied and pasted from 2025's C++ feeler code, which was copied from 2024's feat/astar_rewrite_v3 branch)
        String line;

        //FIXME does this need to be like a .GetFileRelativeToRoot() or something?
        using (StreamReader waypointsFile = new(FeelerConfig.WaypointsFilename))
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

        Logger.LogInformation("Number of waypoints read: " + numWaypoints);

        if (numWaypoints < 1)
        {
            Logger.LogWarning("No waypoints read! Feelers will not use GPS information!");
        }

        return Task.CompletedTask;
    }

    private Task BuildFeelers()
    {
        // reset feelers
        _feelers.Clear();

        var start_angle = new Angle(FeelerConfig.AngleOffset, false);
        var end_angle = new Angle(FeelerConfig.AngleOffset + FeelerConfig.AngularWidth, false);
        var angle_increment = new Angle(FeelerConfig.AngularWidth / FeelerConfig.NumFeelers, false);

        // default to blue FIXME we want lerp to be automatic right?
        var defaultColor = new Scalar(100, 100, 200);

        for (var angle = start_angle; angle < end_angle; angle += angle_increment)
        {
            int x = (int)(FeelerConfig.MaxLength * Math.Cos(angle.To(AngleUnit.Radians)));
            int y = (int)(FeelerConfig.MaxLength * Math.Sin(angle.To(AngleUnit.Radians)));

            _feelers.Add(new Feeler(new SCR_Point(x, y), defaultColor));
        }

        // build some feelers on the other side of the cone/arc formed from start_angle to end_angle
        if (FeelerConfig.BalanceFeelers)
        {
            var flipped_start = (start_angle + new Angle(180, false)).WrapAngle();
            var flipped_end = (flipped_start + new Angle(FeelerConfig.AngularWidth, false)).WrapAngle();

            for (var angle = flipped_start; angle < flipped_end; angle += angle_increment)
            {
                int x = (int)(FeelerConfig.MaxLength * Math.Cos(angle.To(AngleUnit.Radians)));
                int y = (int)(FeelerConfig.MaxLength * Math.Sin(angle.To(AngleUnit.Radians)));

                _feelers.Add(new Feeler(new SCR_Point(x, y), defaultColor));
            }
        }

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
    private Task OnDebugImageReceived(ImageFrame frame, CancellationToken token)
    {
        _debugFrameChannel.Writer.TryWrite(frame);
        return Task.CompletedTask;
    }
    private Task OnMaskReceived(ImageFrame frame, CancellationToken token)
    {
        _maskFrameChannel.Writer.TryWrite(frame);
        return Task.CompletedTask;
    }

    private Task OnPositionReceived(VectorNavReport msg, CancellationToken token)
    {
        _position = msg;

        // if we haven't set a timestamp yet, but have started the run
        if (_runStartTime == 0 && Robot.Instance.State.MotionAllowed && Robot.Instance.State.Mode == RobotModeEnum.Autonomous)
        {
            _runStartTime = TimeUtils.Now(); // then set the timestamp for the start of the run
            _startGpsPos = new LatLng(msg.Latitude, msg.Longitude);
            Logger.LogInformation("Starting Run!"); //FIXME maybe this should just be in OnSystemModeUpdated() or something?
        }

        // if, however, we have set a timestamp, and we're past the GPS wait time, BUT haven't set a direction yet
        else if (_runStartTime != 0 && ((TimeUtils.Now() - _runStartTime) > FeelerConfig.GpsWaitTime) && _direction == "")
        {
            // then pick a set of waypoints based on which direction we are heading

            // FIXME add a set of practice waypoints which we can choose if we don't start like, on the course
            // i.e. our starting GPS position isn't on the actual course
            // also FIXME what if instead of having compNorth/compSouth we just reversed the direction the waypoints went
            // we could do like a int waypointDirection = -1; and set waypointIndex = waypoints.length - 1; or something
            // also FIXME we should move waypoint reading, publishing, popping/reaching, and such to its own subsystem
            // or make it part of the vectornav subsystem. I don't know if I like it being part of Feelers, because I think it leads to
            // identical code duplication in like, whenever we write Smellification/A*

            double heading_degrees = LatLng.TravelHeading(_startGpsPos, new LatLng(msg.Latitude, msg.Longitude)).Value.To(AngleUnit.Degrees);
            if (120 < heading_degrees && heading_degrees < 240)
            {
                _direction = "compSouth";
                Logger.LogInformation("Picking south waypoints!");
            }
            else
            {
                _direction = "compNorth";
                Logger.LogInformation("Picking north waypoints!");
            }

            //TODO: we should flash safety lights to let operator know that the GPS waypoints are working / have been selected
        }

        // waypoint reach detection
        else if (_direction != "" && _waypointsDict.Count() != 0)
        {
            var current_gps = new LatLng(msg.Latitude, msg.Longitude);
            _goalPoint = _waypointsDict[_direction][_waypointIndex];

            var dist = current_gps.Distance(_goalPoint);

            // if we are close enough to the waypoint, and we aren't going to cause an out-of-bounds index error
            if (dist.To(DistanceUnit.Meters) < FeelerConfig.WaypointPopDist && _waypointIndex < (_waypointsDict[_direction].Count - 2))
            {
                if (_waypointTimeStart == 0)
                {
                    // start the clock for how long we have to be close to it
                    _waypointTimeStart = TimeUtils.Now();
                }
                else if ((TimeUtils.Now() - _waypointTimeStart) > FeelerConfig.WaypointPopTime)
                {
                    // then go to the next waypoint
                    _waypointIndex++;

                    Logger.LogInformation("Waypoint Reached! Heading to next...");
                    _waypointTimeStart = 0;

                    // and do the classic green flash for GPS waypoint reached
                    canbus.SafetyLights.FlashTemporary(ColorUtils.Color.Green, token, length: 2000);
                }
            }
        }

        return Task.CompletedTask;
    }

    /**
     * TODO FIXME
     */
    private async Task PerformFeelers(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                // check if we're in autonomous to avoid conflicting with manual control if it's running
                if (Robot.Instance.State.Mode == RobotModeEnum.Autonomous)
                {
                    canbus.SafetyLights.SetAutonomous();

                    if (State == SubsystemState.Operating)
                    {
                        //TODO these might need to be moved out of here if they don't update fast enough for the watchdog on the motor control code...
                        // either that or update canbus.MotorControl to continually send the last motor command, although that has safety issues of its own...
                        var debugFrame = await _debugFrameChannel.Reader.ReadAsync(token);
                        var maskFrame = await _maskFrameChannel.Reader.ReadAsync(token);

                        var debugImg = CvUtils.AsMat(debugFrame);
                        var mask = CvUtils.AsMat(maskFrame);

                        // make the master Feeler that will actually dictate the robot's direction
                        var controlFeeler = new Feeler(new SCR_Point(), new Scalar(200, 200, 0));

                        // bias all Feelers forwards
                        var forwardFeeler = new Feeler(new SCR_Point(0, FeelerConfig.ForwardBiasWeight));
                        foreach (var feeler in _feelers)
                        {
                            feeler.Bias(forwardFeeler * feeler);
                        }

                        // bias towards GPS
                        if (_goalPoint != null)
                        {
                            var current_gps = new LatLng(_position.Latitude, _position.Longitude);

                            var dist = current_gps.Distance(_goalPoint).To(DistanceUnit.Meters);
                            var headingError = GeoUtils.EstimateHeading(current_gps, _goalPoint).Value.To(AngleUnit.Degrees);

                            _gpsFeeler = new Feeler(new SCR_Point(dist, headingError), new Scalar(150, 235, 150));

                            // calculate gps bias for every feeler
                            foreach (var feeler in _feelers)
                            {
                                feeler.Bias(_gpsFeeler * feeler);
                            }
                        }

                        // perform feeler obstacle detection
                        foreach (var feeler in _feelers)
                        {
                            //TODO this could be multithreaded or something (if it's that big of a performance hit, that is)
                            feeler.Update(mask);
                            controlFeeler += feeler;
                        }

                        // if we are allowed to move (earlier check means we are already in auto and operating, so don't have to recheck those)
                        if (Robot.Instance.State.MotionAllowed)
                        {
                            // convert headingArrow to motor output
                            canbus.MotorControl.SendCommand(
                                (float)Math.Clamp(_drivingPID.Calculate(controlFeeler.Current.Y), -FeelerConfig.MaxDriveSpeed, FeelerConfig.MaxDriveSpeed),
                                (float)0.0, //FIXME we need to figure out strafing
                                (float)Math.Clamp(_headingPID.Calculate(controlFeeler.Current.X), -FeelerConfig.MaxTurnSpeed, FeelerConfig.MaxTurnSpeed)
                                );
                        }
                        else
                        {
                            // we are not mobility enabled and thus not allowed to move, so publish velocities of 0 for everything
                            canbus.MotorControl.SendCommand(
                                0f,
                                0f,
                                0f
                            );
                        }

                        // draw debug image
                        foreach (var feeler in _feelers)
                        {
                            feeler.Draw(debugImg);
                        }
                        _gpsFeeler.Draw(debugImg);
                        controlFeeler.Draw(debugImg);

                        // publish debug image
                        var frameBytes = CvUtils.FromMat(debugImg);
                        var newFrame = MessageConstructor.CreateImageFrame(
                            debugFrame.Width,
                            debugFrame.Height,
                            "debug_feelers",
                            frameBytes
                        );

                        var wrappedFrame = MessageWrapper.From(
                            MessageType.ImageFrame,
                            newFrame.ByteBuffer.ToFullArray()
                        );

                        EventBus.Instance.Publish(new MessageWrapperEvent(wrappedFrame));

                        debugImg.Dispose();
                        mask.Dispose();
                    }
                    else
                    {
                        // don't move the robot
                        canbus.MotorControl.SendCommand(0f, 0f, 0f);
                    }
                }
            }
        }
        catch (OperationCanceledException)
        {
            // Expected on shutdown
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Feeler task crashed");
        }
    }
}