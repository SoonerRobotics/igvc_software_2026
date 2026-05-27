using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Core.Config;

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
        // 10hz update loop
        while (!token.IsCancellationRequested)
        {
            // TODO: Implement
            switch (mGoal.Get())
            {
                case TurnRight:
                    //TODO
                    break;
                case TurnLeft: // Q.3 Left Turn
                    // start
                    // maintains target speed (3-5 mph)
                    // turn left into correct lane
                    // 
                    //TODO
                    break;
                case LaneKeep:
                    //TODO: actually publish a laneMessage from visionSubsystem

                    //TODO: A* and purepursuit

                    canbus?.MotorControl.SetVelocities(ForwardSpeed, 0, 0);

                    //TODO: how long do we stay in this goal?

                    break;
                case ChangeLane:
                    if (mLane == SelfdriveMachine.SelfdriveLane.Left)
                    {
                        mLane.Set(SelfdriveMachine.SelfdriveLane.Right);
                        canbus?.MotorControl.SetVelocities(ForwardSpeed, LaneChangeSpeed, 0);
                    }
                    else
                    {
                        mLane.Set(SelfdriveMachine.SelfdriveLane.Left);
                        canbus?.MotorControl.SetVelocities(ForwardSpeed, -LaneChangeSpeed, 0);
                    }

                    await Task.Delay(LaneChangeMs, token);

                    mGoal.Set(SelfdriveMachine.SelfdriveGoal.LaneKeep);

                    break;
                case StopAtObstacle: // qualifacation Q1, stop within 3 feet
                    if (stopped)
                    {
                        canbus?.MotorControl.SetVelocities(0, 0, 0);
                    }
                    else
                    {
                        //FIXME how do we stay in the lane? we need a LaneKeep() to calculate turn speed
                        canbus?.MotorControl.SetVelocities(ForwardSpeed, 0, 0);
                    }

                    //TODO do we need to switch out of this goal?

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

            await Task.Delay(100, token);
        }
    }

    public Task OnDetectionEvent(YoloDetectionEvent e, CancellationToken token)
    {
        return Task.CompletedTask;
    }
}