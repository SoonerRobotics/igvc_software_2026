
using igvc_csharp.src.selfdrive.actions;
using igvc_csharp.src.subsystems.selfdrive;
using igvc_csharp.Utils;

namespace igvc_csharp.src.Subsystems.selfdrive.actions;

public class LaneChangeAction(SelfdriveLane direction, ulong timeoutMs) : ISelfdriveAction
{
    private ulong _startTime = 0;
    private bool _init = false;

    public bool IsInit()
    {
        return _init;
    }

    public void Init(SelfdriveContext context)
    {
        _startTime = TimeUtils.Now();
        _init = true;
    }

    public void Run(SelfdriveContext context)
    {
        if (direction == SelfdriveLane.Left)
        {
            //TODO: turn left
        }
        else
        {
            //TODO: turn right
        }
    }

    public void End(SelfdriveContext context)
    {
        //TODO: do nothing??? Mat.Dispose()?
    }

    public bool IsFinished(SelfdriveContext context)
    {
        if (_startTime == 0)
        {
            return false;
        }

        return (TimeUtils.Now() - _startTime) > timeoutMs;
    }
}