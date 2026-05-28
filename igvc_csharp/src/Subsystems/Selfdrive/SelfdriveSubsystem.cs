using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Core.Config;
using igvc_csharp.subsytems.selfdrive.SelfdriveMachine;

[Subsystem("SelfdriveSubsystem", Disabled = true)]
public class SelfdriveSubsystem(
    CanbusSubsystem? canbus
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

    // [Config("selfdrive.qualification_mode")]
    // public bool QualificationMode = false;

    public override Task Init(CancellationToken token)
    {
        Subscribe<YoloDetectionEvent>(OnDetectionEvent, token);

        //TODO: subscribe to filtered vision
        //TODO: make an A* and PurePursuit for lanekeeping

        _ = Task.Run(() => SelfdriveTask(token), token);

        return Task.CompletedTask;
    }

    private async Task SelfdriveTask(CancellationToken token)
    {
        var TurnRightAction = new SequentialActionGroup(
            new LaneKeepAction(SelfdriveMachine.SelfdriveObstacle.WhiteLine, SelfdriveMachine.SelfdriveLane.Right),
            // new StopAtObstacleAction(SelfdriveMachine.SelfdriveObstacle.WhiteLine),
            new TimedTurnAction(turningSpeed, turningTime), //TODO make this distance based
            new LaneKeepAction(SelfdriveMachine.SelfdriveObstacle.Barrel, SelfdriveMachine.SelfdriveLane.Right)
        );

        var TurnLeftAction = new SequentialActionGroup(
            new LaneKeepAction(SelfdriveMachine.SelfdriveObstacle.WhiteLine, SelfdriveMachine.SelfdriveLane.Left),
            // new StopAtObstacleAction(stop sign),
            new TimedTurnAction(turningSpeed, turningTime), //TODO make this distance based
            new LaneKeepAction(SelfdriveMachine.SelfdriveObstacle.Obstacle, SelfdriveMachine.SelfdriveLane.Left)
        );

        var laneKeepAction = new LaneKeepAction(until barrel, lane.Any);

        var ChangeLaneAction = new LaneChangeAction();

        var StopAtObstacle = new StopAction(full stop);
        
        var TimedStopAtObstacle = new StopAction(until timed, 3000);
        
        var StopAtObstacleUntilClear = new StopAction(until clear);

        var LaneChangeAtObstacle = new SequentialActionGroup(
            //TODO idk what to do here how this one works
        );

        // 10hz update loop
        while (!token.IsCancellationRequested)
        {
            SequentialActionGroup action;
            // TODO: Implement
            switch (mGoal.Get())
            {
                case TurnRight:
                    action = TurnRightAction; //FIXME is this gonna work right?
                case TurnLeft:
                    //TODO
                    // break;
                case LaneKeep:
                    //TODO
                    break;
                case ChangeLane:
                    //TODO
                    // break;
                case StopAtObstacle:
                    //TODO
                    break;
                case TimedStopAtObstacle:
                    //TODO
                    break;
                case StopAtObstacleUntilClear:
                    //TODO
                    break;
                case LaneChangeAtObstacle:
                    //TODO
                    break;
                default:
                    canbus?.MotorControl.SetVelocities(0, 0, 0);
                    break;
            }

            SelfdriveContext ctx;

            //TODO: fill context with data and stuff

            if (!action.Init)
            {
                action.Init();
            }

            action.Run(ctx);

            if (action.IsFinished())
            {
                action.End();

                action = null; //FIXME what do we do here? default action?
            }

            await Task.Delay(100, token);
        }
    }

    public Task OnDetectionEvent(YoloDetectionEvent e, CancellationToken token)
    {
        return Task.CompletedTask;
    }
}