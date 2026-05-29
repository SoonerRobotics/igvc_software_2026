
using igvc_csharp.src.Subsystems.selfdrive;

namespace igvc_csharp.src.Subsystems.selfdrive;

public class StopAction : ISelfdriveAction
{
    public StopAction(until condition)
    {

    }

    public override void Init(SelfdriveContext context)
    {
        //TODO: ???
    }

    public override void Run(SelfdriveContext context)
    {
        //TODO: calculate motor outputs to keep us centered in current lane while waiting for an obstacle?
    }

    public override void End(SelfdriveContext context)
    {
        //TODO: do nothing??? Mat.Dispose()?
    }

    public override bool IsFinished(SelfdriveContext context)
    {
        return condition == true;
    }
}