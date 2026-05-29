
using igvc_csharp.src.selfdrive.actions;
using igvc_csharp.src.Subsystems.selfdrive;

namespace igvc_csharp.src.Subsystems.selfdrive;

public class StopAction() : ISelfdriveAction
{
    private bool _init = false;

    public bool IsInit()
    {
        return _init;
    }

    public void Init(SelfdriveContext context)
    {
        _init = true;
        //TODO: ???
    }

    public void Run(SelfdriveContext context)
    {
        //TODO: calculate motor outputs to keep us centered in current lane while waiting for an obstacle?
    }

    public void End(SelfdriveContext context)
    {
        //TODO: do nothing??? Mat.Dispose()?
    }

    public bool IsFinished(SelfdriveContext context)
    {
        // return condition == true;
        return true;
    }
}