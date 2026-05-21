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
using igvc_csharp.src.Utils;


namespace igvc_csharp.src.Subsystems;

[Subsystem("FeelerSubsystem", Disabled = false)]
public class FeelerSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    // actual feeler stuff
    private List<Feeler> _feelers = [];
    private Feeler _gpsFeeler = new();

    // pid controllers
    private PIDController _drivingPID = new(
        FeelerConfig.DriveKp,
        FeelerConfig.DriveKi,
        FeelerConfig.DriveKd
    );
    //FIXME we actually could want to like, no-rotate and only strafe, so we'd need to headingPID towards waypoints but also strafePID to avoid obstacles as well
    private PIDController _headingPID = new(
        FeelerConfig.HeadingKp,
        FeelerConfig.HeadingKi,
        FeelerConfig.HeadingKd
    );

    // GPS stuff
    private VectornavReport _position;
    private LatLng? _goalPoint;

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

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        BuildFeelers();

        // subscribers
        Subscribe<ConfigChangedEvent>(OnConfigChanged, token);

        SubscribeImage(
            "combined_view",
            OnDebugImageReceived,
            token
        );

        SubscribeImage(
            "combined_filtered",
            OnMaskReceived,
            token
        );

        SubscribeMessage<VectornavReport>(
            MessageType.VectorNav,
            OnPositionReceived,
            token
        );

        SubscribeMessage<Waypoint>(
            MessageType.Waypoint,
            OnWaypointReceived,
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

    private void ResetPIDs()
    {
        _drivingPID = new(
            FeelerConfig.DriveKp,
            FeelerConfig.DriveKi,
            FeelerConfig.DriveKd
        );

        _headingPID = new(
           FeelerConfig.HeadingKp,
           FeelerConfig.HeadingKi,
           FeelerConfig.HeadingKd
       );
    }

    private Task OnConfigChanged(ConfigChangedEvent e, CancellationToken token)
    {
        if (!e.Path.StartsWith("feelers"))
        {
            return Task.CompletedTask;
        }

        // build new feelers
        BuildFeelers();

        // get new PID constants
        ResetPIDs();

        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        //FIXME should we be in charge of this? I think canbus could handle this automatically...
        if (old.Mode == RobotModeEnum.Autonomous && updated.Mode != RobotModeEnum.Autonomous)
        {
            canbus.SafetyLights.SetAutonomous();
        }
        else if (updated.Mode == RobotModeEnum.Autonomous)
        {
            canbus.SafetyLights.SetAutonomous();
        }
        else if (updated.Mode == RobotModeEnum.Manual)
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

        SetOperatingState(SubsystemState.Operating);

        return Task.CompletedTask;
    }

    private Task OnPositionReceived(VectornavReport msg, CancellationToken token)
    {
        _position = msg;

        return Task.CompletedTask;
    }

    private Task OnWaypointReceived(Waypoint msg, CancellationToken token)
    {
        _goalPoint = new(msg.Latitude, msg.Longitude);

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
                if (!FeelerConfig.UseFeelers)
                {
                    //FIXME is there something better to do here? Like transition to shutdown and find a way to come back? or just trut in someone restarting the code?
                    await Task.Delay(5000, token);
                    continue;
                }

                // check if we're in autonomous to avoid conflicting with manual control if it's running
                if (BaseRobot.Instance.State.Mode == RobotModeEnum.Autonomous)
                {
                    //FIXME should we be setting safetyLights or should it be setting automatically?
                    canbus.SafetyLights.SetAutonomous();

                    if (State == SubsystemState.Operating)
                    {
                        //TODO these might need to be moved out of here if they don't update fast enough for the watchdog on the motor control code...
                        // either that or update canbus.MotorControl to continually send the last motor command, although that has safety issues of its own...
                        var debugFrame = await _debugFrameChannel.Reader.ReadAsync(token);
                        var maskFrame = await _maskFrameChannel.Reader.ReadAsync(token);

                        var debugImg = CvUtils.AsMat(debugFrame);
                        var mask = CvUtils.AsMat(maskFrame);

                        // zero the mask to ignore all obstacles
                        if (FeelerConfig.UseOnlWaypoints)
                        {
                            mask.SetTo(0);
                        }

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
                        if (BaseRobot.Instance.State.MotionAllowed)
                        {
                            // convert headingArrow to motor output
                            canbus.MotorControl.SetVelocities(
                                (float)Math.Clamp(_drivingPID.Calculate(controlFeeler.Current.Y), -FeelerConfig.MaxDriveSpeed, FeelerConfig.MaxDriveSpeed),
                                (float)0.0, //FIXME we need to figure out strafing
                                (float)Math.Clamp(_headingPID.Calculate(controlFeeler.Current.X), -FeelerConfig.MaxTurnSpeed, FeelerConfig.MaxTurnSpeed)
                                );
                        }
                        else
                        {
                            // we are not mobility enabled and thus not allowed to move, so publish velocities of 0 for everything
                            canbus.MotorControl.SetVelocities(
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
                        canbus.MotorControl.SetVelocities(0f, 0f, 0f);
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