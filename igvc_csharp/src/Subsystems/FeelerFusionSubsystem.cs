using igvc_csharp.Core;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Utils;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems;

[Subsystem("FeelerFusionSubsystem", DependsOn=[typeof(ControllerSubsystem)])]
public class FeelerFusionSubsystem(ControllerSubsystem controller, CanbusSubsystem canbus) : SubsystemBase
{
    //FIXME
    public override Task Init(CancellationToken token)
    {
        return Task.CompletedTask;
    }

    //TODO: pub and sub and all that
}