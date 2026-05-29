using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Core.Config;
using igvc_csharp.subsytems.selfdrive.SelfdriveMachine;

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
    [Config("selfdrive.lane_change_ms")]
    public int LaneChangeMs = 2000; //FIXME ideally this is distance-based (or actually vision-based)
    [Config("selfdrive.obstacle_stop_ms")]
    public int ObstacleStopMs = 5000; //FIXME ideally this is distance-based (or actually vision-based)


    public SubsystemProperty<SelfdriveMachine.SelfdriveGoal> mGoal = new SubsystemProperty<SelfdriveMachine.SelfdriveGoal>("goal", SelfdriveMachine.SelfdriveGoal.LaneKeep);
    public SubsystemProperty<SelfdriveMachine.SelfdriveLane> mLane = new SubsystemProperty<SelfdriveMachine.SelfdriveLane>("lane", SelfdriveMachine.SelfdriveLane.Right);
    // public SubsystemProperty<SelfdriveMachine.SelfdriveTest> mTest = new SubsystemProperty<SelfdriveMachine.SelfdriveTest>("functional_test", SelfdriveMachine.SelfdriveTest.PedestrianDetection_FI_1);
    public SubsystemProperty<SelfdriveMachine.QualificationTest> mTest = new SubsystemProperty<SelfdriveMachine.QualificationTest>("qualification_test", SelfdriveMachine.SelfdriveTest.LaneKeeping_Q1);

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
        SubscribeImage(
            "combined_debug",
            OnDebugImageReceived,
            token
        );

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
            // TODO: Implement
            // switch (mGoal.Get())
            // {
            //     case TurnRight:
            //         action = TurnRightAction; //FIXME is this gonna work right?
            //     case TurnLeft:
            //         action = TurnLeftAction;
            //     case LaneKeep:
            //         action = laneKeepAction;
            //     case ChangeLane:
            //         action = ChangeLaneAction;
            //     case StopAtObstacle:
            //         //TODO
            //         break;
            //     case TimedStopAtObstacle:
            //         //TODO
            //         break;
            //     case StopAtObstacleUntilClear:
            //         //TODO
            //         break;
            //     case LaneChangeAtObstacle:
            //         //TODO
            //         break;
            //     default:
            //         canbus?.MotorControl.SetVelocities(0, 0, 0);
            //         break;
            // }

            ISelfdriveAction action;
            switch (mTest.Get())
            {
                case LineDetection_Q2: // literally just display the annotated debug image
                case LaneKeeping_Q1:
                    action = new SequentialAction(
                        new LaneKeepAction(SelfdriveMachine.Lane.Right, SelfdriveMachine.Obstacle.Barrel)
                    );
                    break;

                case LeftTurn_Q3:
                    action = new SequentialAction(
                        new TurnAction(ForwardSpeed*2, 0f, -TurnSpeed, 5000),
                        new LaneKeepAction(SelfdriveMachine.Lane.Right, SelfdriveMachine.Obstacle.Barrel)
                    );
                    break;

                case RightTurn_Q4:
                    action = new SequentialAction(
                        new TurnAction(ForwardSpeed, 0f, TurnSpeed, 3000),
                        new LaneKeepAction(SelfdriveMachine.Lane.Right, SelfdriveMachine.Obstacle.Barrel)
                    );
                    break;
                default:
                    //FIXME
                    break;
            }

            var ctx = context.Clone();

            if (!action.Init)
            {
                action.Init(ctx);
            }

            action.Run(ctx);

            if (action.IsFinished())
            {
                action.End(ctx);

                action = null; //FIXME what do we do here? default action?
            }

            ctx.Dispose(); //???

            //TODO: dispose regular context Mats?

            await Task.Delay(100, token);
        }
    }

    public Task OnDetectionEvent(YoloDetectionEvent e, CancellationToken token)
    {
        return Task.CompletedTask;
    }
}