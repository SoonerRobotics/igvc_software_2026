using igvc_csharp.Core;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("VectorNavSubsystem")]
public class VectorNavSubsystem : SubsystemBase
{
    public override Task Init(CancellationToken token)
    {
        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        return Task.CompletedTask;
    }
}