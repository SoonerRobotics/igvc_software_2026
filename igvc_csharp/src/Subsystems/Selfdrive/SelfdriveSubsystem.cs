using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Core.Config;
using igvc_csharp.Subsystems.Navigation;
using igvc_csharp.src.Subsystems.selfdrive;
using igvc_csharp.src.selfdrive.actions;
using igvc_csharp.src.Subsystems.selfdrive.actions;

namespace igvc_csharp.src.subsystems.selfdrive;

[Subsystem("SelfdriveSubsystem", Disabled = true)]
public class SelfdriveSubsystem(
    CanbusSubsystem? canbus,
    NavigationSubsystem navigation
) : SubsystemBase
{
    // === configuration ===
    //FIXME make these like, actual Meters per Second?
    [Config("selfdrive.forward_speed")]
    public double ForwardSpeed = 1.0; // default forward speed FIXME target selfdrive speed is 4-5 MPH I think? according to the rules?
    [Config("selfdrive.lane_change_speed")]
    public double LaneChangeSpeed = 0.4; // positive is to the right
    [Config("selfdrive.sideways_speed")]
    public double SidewaysSpeed = 0.2; // positive is to the left
    [Config("selfdrive.turn_speed")]
    public double TurnSpeed = 0.4; // positive is to the left
    [Config("selfdrive.lane_change_ms")]
    public int LaneChangeMs = 2000; //FIXME ideally this is distance-based (or actually vision-based)
    [Config("selfdrive.obstacle_stop_ms")]
    public int ObstacleStopMs = 5000; //FIXME ideally this is distance-based (or actually vision-based)


    public SubsystemProperty<SelfdriveGoal> mGoal = new SubsystemProperty<SelfdriveGoal>("goal", SelfdriveGoal.LaneKeep);
    public SubsystemProperty<SelfdriveLane> mLane = new SubsystemProperty<SelfdriveLane>("lane", SelfdriveLane.Right);
    public SubsystemProperty<SelfdriveTest> mTest = new SubsystemProperty<SelfdriveTest>("functional_test", SelfdriveTest.PedestrianDetection_FI_1);
    public SubsystemProperty<QualificationTest> mQualTest = new SubsystemProperty<QualificationTest>("qualification_test", QualificationTest.LaneKeeping_Q1);

    [Config("selfdrive.qualification_mode")]
    public bool QualificationMode = false;

    // PedestrianDetection_FI_1,
    // TireDetection_FI_2,
    // StopSignDetection_FII_1,
    // LaneKeeping_FIII_1,
    // LeftTurn_FIII_2,
    // RightTurn_FIII_3,
    // PullOut_FIV_1,
    // PullIn_FIV_2,
    // ParallelPark_FIV_3,
    // StaticPedestrian_FV_1,
    // DynamicPedestrian_FV_2,
    // AvoidPedestrian_FV_3,
    // BarrelLaneChange_FV_4,
    // CurvedLaneKeeping_FVI_1,
    // CurvedLaneChange_FVI_2,
    // PotholeDetection_FVII_1

    private SelfdriveContext context = new();

    public override Task Init(CancellationToken token)
    {
        Subscribe<YoloDetectionEvent>(OnDetectionEvent, token);
        // === subscribers ===
        // SubscribeImage(
        //     "combined_debug",
        //     OnDebugImageReceived,
        //     token
        // );

        // for A* TODO I don't think we actually need this?
        // SubscribeImage(
        //     "combined_inflated",
        //     OnInflatedReceived
        // );

        // shouldn't need GPS ?
        // SubscribeMessage<VectornavReport>(
        //     MessageType.VectorNav,
        //     OnPositionReceived,
        //     token
        // );

        // SubscribeMessage<Waypoint>(
        //     MessageType.Waypoint,
        //     OnWaypointReceived,
        //     token
        // );

        _ = Task.Run(() => SelfdriveTask(token), token);

        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        //FIXME do something here?

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
                switch (mTest.Get())
                {
                    //TODO: should the static tests have actions? like a StaticTestAction that just sits there?
                    // because we're publishing the debug images regardless, right? but we shouldn't during a real run for performance?
                    case SelfdriveTest.PedestrianDetection_FI_1:
                        action = new SequentialAction([
                            // no action, static test
                        ]);
                        break;
                    case SelfdriveTest.TireDetection_FI_2:
                        action = new SequentialAction([
                            // no action, static test
                        ]);
                        break;
                    case SelfdriveTest.StopSignDetection_FII_1:
                        action = new SequentialAction([
                            // no action, static test
                        ]);
                        break;
                    case SelfdriveTest.LaneKeeping_FIII_1:
                        action = new SequentialAction([
                            new LaneKeepAction(SelfdriveLane.Right, SelfdriveObstacles.Stopsign),
                            //FIXME we might need to insert a DriveStraightTimed here to make it through the intersection?
                            new LaneKeepAction(SelfdriveLane.Right, SelfdriveObstacles.Barrel)
                        ]);
                        break;
                    case SelfdriveTest.LeftTurn_FIII_2:
                        action = new SequentialAction([
                            new LaneKeepAction(SelfdriveLane.Right, SelfdriveObstacles.Stopsign),
                            //FIXME might need a WaitTimeout() so it's a "full" stop
                            new TimedDriveAction(ForwardSpeed*2, 0, TurnSpeed, 5000),
                            new LaneKeepAction(SelfdriveLane.Right, SelfdriveObstacles.Barrel)
                        ]);
                        break;
                    case SelfdriveTest.RightTurn_FIII_3:
                        action = new SequentialAction([
                            new LaneKeepAction(SelfdriveLane.Right, SelfdriveObstacles.Stopsign),
                            //FIXME might need a WaitTimeout() so it's a "full" stop
                            new TimedDriveAction(ForwardSpeed, 0, -TurnSpeed, 3000),
                            new LaneKeepAction(SelfdriveLane.Right, SelfdriveObstacles.Barrel)
                        ]);
                        break;
                    case SelfdriveTest.PullOut_FIV_1:
                        action = new SequentialAction([
                            new TimedDriveAction(ForwardSpeed, 0, TurnSpeed*PullOutDirection, 5000),
                            new LaneKeepAction(SelfdriveLane.Right, SelfdriveObstacles.Barrel)
                        ]);
                        break;
                    case SelfdriveTest.PullIn_FIV_2:
                        action = new SequentialAction([
                            //TODO FIXME not sure how we're doing this one?
                        ]);
                        break;
                    case SelfdriveTest.ParallelPark_FIV_3:
                        action = new SequentialAction([
                            //FIXME we should do like, actual lane/line detection for this one
                            new TimedDriveAction(-ForwardSpeed, 0, 0, 3000),
                            new TimedDriveAction(0, -SidewaysSpeed, 0, 1000)
                        ]);
                        break;
                    case SelfdriveTest.StaticPedestrian_FV_1:
                        action = new SequentialAction([
                            // no action, static test
                        ]);
                        break;
                    case SelfdriveTest.DynamicPedestrian_FV_2:
                        action = new SequentialAction([
                            new LaneKeepAction(SelfdriveLane.Left, SelfdriveObstacles.Pedestrian),
                            //TODO: might want a TimedWaitAction() for a full stop here?
                            //TODO: need a WaitForClear here
                            new LaneKeepAction(SelfdriveLane.Left, SelfdriveObstacles.Barrel)
                        ]);
                        break;
                    case SelfdriveTest.AvoidPedestrian_FV_3:
                        action = new SequentialAction([
                            new LaneKeepAction(SelfdriveLane.Right, SelfdriveObstacles.Pedestrian),
                            new LaneChangeAction(SelfdriveLane.Left, 3000), //FIXME this can be replaced by a TimedDriveAction
                            //also FIXME don't need to pass it the lane to change into, it's always the opposite of thel lane you're in, obviously
                            new LaneKeepAction(SelfdriveLane.Left, SelfdriveObstacles.Barrel)
                        ]);
                        break;
                    case SelfdriveTest.BarrelLaneChange_FV_4:
                        action = new SequentialAction([

                        ]);
                        break;
                    case SelfdriveTest.CurvedLaneKeeping_FVI_1:
                        action = new SequentialAction([

                        ]);
                        break;
                    case SelfdriveTest.CurvedLaneChange_FVI_2:
                        action = new SequentialAction([

                        ]);
                        break;
                    case SelfdriveTest.PotholeDetection_FVII_1:
                        action = new SequentialAction([

                        ]);
                        break;

                    default:
                        canbus?.MotorControl.SetVelocities(0, 0, 0);
                        break;
                }
            }
            else
            {


                switch (mQualTest.Get())
                {
                    case QualificationTest.LineDetection_Q2: // literally just display the annotated debug image
                    case QualificationTest.LaneKeeping_Q1:
                        action = new SequentialAction(
                            [new LaneKeepAction(SelfdriveLane.Right, SelfdriveObstacles.Barrel)
                        ]);
                        break;

                    case QualificationTest.LeftTurn_Q3:
                        action = new SequentialAction([
                            new TimedDriveAction(ForwardSpeed*2, 0f, TurnSpeed, 5000),
                        new LaneKeepAction(SelfdriveLane.Right, SelfdriveObstacles.Barrel)
                        ]);
                        break;

                    case QualificationTest.RightTurn_Q4:
                        action = new SequentialAction([
                            new TimedDriveAction(ForwardSpeed, 0f, -TurnSpeed, 3000),
                        new LaneKeepAction(SelfdriveLane.Right, SelfdriveObstacles.Barrel)
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
                action.Init(ctx);
            }

            action.Run(ctx);

            if (action.IsFinished(ctx))
            {
                action.End(ctx);

                action = null; //FIXME what do we do here? default action?
            }

            // ctx.Dispose(); //???

            //TODO: dispose regular context Mats?

            await Task.Delay(100, token);
        }
    }

    public Task OnDetectionEvent(YoloDetectionEvent e, CancellationToken token)
    {
        return Task.CompletedTask;
    }
}