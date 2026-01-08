using igvc_csharp.Core;

namespace igvc_csharp.Subsystems;

[Subsystem("VisionSubsystem")]
public class VisionSubsystem : ISubsystem
{
    public Task Init(CancellationToken token)
    {
        return Task.CompletedTask;
    }

    public Task Periodic(CancellationToken token)
    {
        return Task.CompletedTask;
    }

    public Task Shutdown()
    {
        return Task.CompletedTask;
    }

    public Task Restart()
    {
        return Task.CompletedTask;
    }
}