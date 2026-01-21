using igvc_csharp.Core;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("RealsenseSubsystem", DependsOn = [], Disabled = false)]
public class RealsenseSubsystem : SubsystemBase
{
    public override Task Init(CancellationToken token)
    {
        return Task.CompletedTask;
    }

    public override Task Restart()
    {
        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        return Task.CompletedTask;
    }
}