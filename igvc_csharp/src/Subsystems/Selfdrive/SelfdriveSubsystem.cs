using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;

[Subsystem("SelfdriveSubsystem", Disabled = true)]
public class SelfdriveSubsystem(
    CanbusSubsystem? canbus
) : SubsystemBase
{
    public SubsystemProperty<SelfdriveMachine.SelfdriveGoal> mGoal = new SubsystemProperty<SelfdriveMachine.SelfdriveGoal>("goal", SelfdriveMachine.SelfdriveGoal.LaneKeep);
    public SubsystemProperty<SelfdriveMachine.SelfdriveLane> mLane = new SubsystemProperty<SelfdriveMachine.SelfdriveLane>("lane", SelfdriveMachine.SelfdriveLane.Right);

    public override Task Init(CancellationToken token)
    {
        Subscribe<YoloDetectionEvent>(OnDetectionEvent, token);
        
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