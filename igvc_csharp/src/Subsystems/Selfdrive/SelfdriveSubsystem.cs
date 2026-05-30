using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Core.Config;
using igvc_csharp.Subsystems.Navigation;
using igvc_csharp.src.Subsystems.selfdrive;
using igvc_csharp.src.selfdrive.actions;
using igvc_csharp.src.Subsystems.selfdrive.actions;
using igvc_csharp.Core.Units;
using Messages;
using igvc_csharp.Utils;

namespace igvc_csharp.src.subsystems.selfdrive;

[Subsystem("SelfdriveSubsystem", Disabled = true)]
public class SelfdriveSubsystem(
    CanbusSubsystem? canbus
) : SubsystemBase
{
    [Config("selfdrive.forward_speed")]
    public LinearVelocity ForwardSpeed = LinearVelocityUnit.MilesPerHour.Of(3);

    [Config("selfdrive.lane_change_speed")]
    public LinearVelocity LaneChangeSpeed = LinearVelocityUnit.MilesPerHour.Of(0.5);

    [Config("selfdrive.turn_speed")]
    public AngularVelocity TurnSpeed = AngularVelocityUnit.DegreesPerSecond.Of(30);

    [Config("selfdrive.feet_to_stop")]
    public Distance ObstacleStopDistnace = DistanceUnit.Feet.Of(2);

    [Config("selfdrive.pull_out_direction")]
    public int PullOutDirection = 0; // positive is to the left

    [Config("selfdrive.obstacle_reaction_distance")]
    public Distance ObstacleReactionDistance = DistanceUnit.Feet.Of(12);

    [Config("selfdrive.lane_change_timeout")]
    public ulong laneChangeTimeoutMs = 3000;

    public SubsystemProperty<SelfdriveGoal> mGoal = new SubsystemProperty<SelfdriveGoal>("goal", SelfdriveGoal.LaneKeep);
    public SubsystemProperty<SelfdriveLane> mLane = new SubsystemProperty<SelfdriveLane>("lane", SelfdriveLane.Right);
    public SubsystemProperty<SelfdriveTest> mTest = new SubsystemProperty<SelfdriveTest>("functional_test", SelfdriveTest.PedestrianDetection_FI_1);
    public SubsystemProperty<QualificationTest> mQualTest = new SubsystemProperty<QualificationTest>("qualification_test", QualificationTest.LaneKeeping_Q1);

    [Config("selfdrive.qualification_mode")]
    public bool QualificationMode = false;

    private SelfdriveContext context = new();
    // private lock object contextLock = new();
    private RobotPosition? mLastPosition = null;

    public override Task Init(CancellationToken token)
    {
        context = new()
        {
            canbus = canbus!,
            whiteLaneDistance = 0,
            lastZedFrame = null,
            lastCenterFrame = null
        };

        Subscribe<YoloDetectionEvent>(OnDetectionEvent, token);
        SubscribeImage("zed", OnZedImageReceived, token);
        SubscribeImage("center", OnCenterImageReceived, token);

        _ = Task.Run(() => SelfdriveTask(token), token);

        return Task.CompletedTask;
    }

    public Task OnZedImageReceived(ImageFrame frame, CancellationToken ct)
    {
        var mat = CvUtils.AsMat(frame);

        // l

        return Task.CompletedTask;
    }

    public Task OnCenterImageReceived(ImageFrame frame, CancellationToken ct)
    {
        var mat = CvUtils.AsMat(frame);
        var lane = CvUtils.ExtractLaneSingle(mat);
        var laneDistance = CvUtils.DistanceToWhiteLane(mat);
        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        return Task.CompletedTask;
    }

    public override void OnPositionChanged(RobotPosition position)
    {
        if (mLastPosition == null)
        {
            mLastPosition = position;
            return;
        }

        mLastPosition = position;
    }

    public Task OnDetectionEvent(YoloDetectionEvent e, CancellationToken token)
    {
        if (!context.YoloDetections.TryAdd(e.label, e))
        {
            context.YoloDetections.Remove(e.label); //FIXME is there any sort of cleanup / Dispose we need to do here?
            context.YoloDetections.Add(e.label, e);
        }

        //TODO: need to like, clear the dictionary once we move to a new action or something...
        // have a _clearEverything flag? maybe clear it whenever we Init() a new Action?

        return Task.CompletedTask;
    }

    private async Task SelfdriveTask(CancellationToken token)
    {
        // 10hz update loop
        while (!token.IsCancellationRequested)
        {
            if (BaseRobot.Instance?.State.Mission != MissionEnum.Selfdrive)
            {
                await Task.Delay(5000, token); // FIXME is this good?
                continue;
            }

            ISelfdriveAction? action = null;
            if (!QualificationMode)
            {
                // switch (mTest.Get())
                // {
                //     //TODO: should the static tests have actions? like a StaticTestAction that just sits there?
                //     // because we're publishing the debug images regardless, right? but we shouldn't during a real run for performance?
                //     case SelfdriveTest.PedestrianDetection_FI_1:
                //         action = new SequentialAction([
                //             // no action, static test
                //         ]);
                //         break;
                //     case SelfdriveTest.TireDetection_FI_2:
                //         action = new SequentialAction([
                //             // no action, static test
                //         ]);
                //         break;
                //     case SelfdriveTest.StopSignDetection_FII_1:
                //         action = new SequentialAction([
                //             // no action, static test
                //         ]);
                //         break;
                //     case SelfdriveTest.LaneKeeping_FIII_1:
                //         action = new SequentialAction([
                //             new LaneKeepAction(SelfdriveObstacles.Stopsign, DistanceUnit.Feet.Of(feetToStop)),
                //             //FIXME we might need to insert a DriveStraightTimed here to make it through the intersection?
                //             new LaneKeepAction(SelfdriveObstacles.Barrel, DistanceUnit.Feet.Of(feetToStop))
                //         ]);
                //         break;
                //     case SelfdriveTest.LeftTurn_FIII_2:
                //         action = new SequentialAction([
                //             new LaneKeepAction(SelfdriveObstacles.Stopsign, DistanceUnit.Feet.Of(feetToStop)),
                //             //FIXME might need a WaitTimeout() so it's a "full" stop
                //             new TimedDriveAction(ForwardSpeed*2, 0, TurnSpeed, 5000),
                //             new LaneKeepAction(SelfdriveObstacles.Barrel, DistanceUnit.Feet.Of(feetToStop))
                //         ]);
                //         break;
                //     case SelfdriveTest.RightTurn_FIII_3:
                //         action = new SequentialAction([
                //             new LaneKeepAction(SelfdriveObstacles.Stopsign, DistanceUnit.Feet.Of(feetToStop)),
                //             //FIXME might need a WaitTimeout() so it's a "full" stop
                //             new TimedDriveAction(ForwardSpeed, 0, -TurnSpeed, 3000),
                //             new LaneKeepAction(SelfdriveObstacles.Barrel, DistanceUnit.Feet.Of(feetToStop))
                //         ]);
                //         break;
                //     case SelfdriveTest.PullOut_FIV_1:
                //         action = new SequentialAction([
                //             new TimedDriveAction(ForwardSpeed, 0, TurnSpeed*PullOutDirection, 5000),
                //             new LaneKeepAction(SelfdriveObstacles.Barrel, DistanceUnit.Feet.Of(feetToStop))
                //         ]);
                //         break;
                //     case SelfdriveTest.PullIn_FIV_2:
                //         action = new SequentialAction([
                //             //TODO FIXME not sure how we're doing this one?
                //         ]);
                //         break;
                //     case SelfdriveTest.ParallelPark_FIV_3:
                //         action = new SequentialAction([
                //             //FIXME we should do like, actual lane/line detection for this one
                //             new TimedDriveAction(-ForwardSpeed, 0, 0, 3000),
                //             new TimedDriveAction(0, -SidewaysSpeed, 0, 1000)
                //         ]);
                //         break;
                //     case SelfdriveTest.StaticPedestrian_FV_1:
                //         action = new SequentialAction([
                //             new LaneKeepAction(SelfdriveObstacles.Pedestrian, DistanceUnit.Feet.Of(6)) //FIXME configurable PedestrianStop distance?
                //         ]);
                //         break;
                //     case SelfdriveTest.DynamicPedestrian_FV_2:
                //         action = new SequentialAction([
                //             new LaneKeepAction(SelfdriveObstacles.Pedestrian, DistanceUnit.Feet.Of(6)),
                //             //TODO: might want a TimedWaitAction() for a full stop here?
                //             //TODO: need a WaitForClear here
                //             new LaneKeepAction(SelfdriveObstacles.Barrel, DistanceUnit.Feet.Of(feetToStop))
                //         ]);
                //         break;
                //     case SelfdriveTest.AvoidPedestrian_FV_3:
                //         action = new SequentialAction([
                //             new LaneKeepAction(SelfdriveObstacles.Pedestrian, DistanceUnit.Feet.Of(11)),
                //             new LaneChangeAction(laneChangeTimeoutMs),
                //             new LaneKeepAction(SelfdriveObstacles.Barrel, DistanceUnit.Feet.Of(feetToStop))
                //         ]);
                //         break;
                //     case SelfdriveTest.BarrelLaneChange_FV_4:
                //         action = new SequentialAction([
                //             //FIXME we need to change lanes within a certain distance, not end within a certain distance?
                //             new LaneKeepAction(SelfdriveObstacles.Barrel, DistanceUnit.Feet.Of(feetToStop)),
                //             new LaneChangeAction(laneChangeTimeoutMs),
                //             new LaneKeepAction(SelfdriveObstacles.Barrel, DistanceUnit.Feet.Of(feetToStop))
                //         ]);
                //         break;
                //     case SelfdriveTest.CurvedLaneKeeping_FVI_1:
                //         action = new SequentialAction([
                //             new LaneKeepAction(SelfdriveObstacles.Barrel, DistanceUnit.Feet.Of(feetToStop))
                //         ]);
                //         break;
                //     case SelfdriveTest.CurvedLaneChange_FVI_2:
                //         action = new SequentialAction([
                //             //FIXME this is just the same as the regular lane change no?
                //         ]);
                //         break;
                //     case SelfdriveTest.PotholeDetection_FVII_1:
                //         action = new SequentialAction([
                //             new LaneKeepAction(SelfdriveObstacles.Pothole, DistanceUnit.Feet.Of(feetToStop)),
                //             new LaneChangeAction(laneChangeTimeoutMs),
                //             new LaneKeepAction(SelfdriveObstacles.Barrel, DistanceUnit.Feet.Of(feetToStop))
                //         ]);
                //         break;

                //     default:
                //         canbus?.MotorControl.SetVelocities(0, 0, 0);
                //         break;
                // }
            }
            else
            {
                switch (mQualTest.Get())
                {
                    // All we do is display the annotated image
                    case QualificationTest.LineDetection_Q2:

                    // Just 
                    case QualificationTest.LaneKeeping_Q1:
                        action = new SequentialAction(
                            [new LaneKeepAction(SelfdriveObstacles.Barrel, ObstacleStopDistnace)
                        ]);
                        break;

                    case QualificationTest.LeftTurn_Q3:
                        action = new SequentialAction([
                            new TimedDriveAction(ForwardSpeed, ForwardSpeed, TurnSpeed, 5000),
                            new LaneKeepAction(SelfdriveObstacles.Barrel, ObstacleStopDistnace)
                        ]);
                        break;

                    case QualificationTest.RightTurn_Q4:
                        action = new SequentialAction([
                            new TimedDriveAction(ForwardSpeed, ForwardSpeed, -TurnSpeed, 3000),
                            new LaneKeepAction(SelfdriveObstacles.Barrel, ObstacleStopDistnace)
                        ]);
                        break;
                    default:
                        //FIXME
                        break;
                }
            }

            // var ctx = context.Clone();
            var ctx = context; // TOOD: Fix
            if (!action?.IsInit() ?? false)
            {
                action?.Init(ctx);
            }

            action?.Run(ctx);

            if (action?.IsFinished(ctx) ?? false)
            {
                action.End(ctx);

                action = null; //FIXME what do we do here? default action?
            }

            //TODO if we are completely out of actions then what?

            // ctx.Dispose(); //???

            //TODO: dispose regular context Mats?

            await Task.Delay(100, token);
        }
    }
}