
using igvc_csharp.src.selfdrive.actions;
using igvc_csharp.src.subsystems.selfdrive;
using igvc_csharp.Utils;

namespace igvc_csharp.src.Subsystems.selfdrive.actions;

public class LaneKeepAction(SelfdriveLane lane, SelfdriveObstacles obstacle, ulong timeout = 0) : ISelfdriveAction
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

        //TODO: make an A* and pure pursuit
    }

    public void Run(SelfdriveContext context)
    {
        //TODO: calculate motor outputs to keep us centered in current lane

        context.canbus.MotorControl.SetVelocities(0, 0, 0); //FIXME
    }

    public void End(SelfdriveContext context)
    {
        //TODO: do nothing??? Mat.Dispose()?

        //FIXME we could have like a "StopOnEnd" field in the constructor?
        context.canbus.MotorControl.SetVelocities(0, 0, 0);
    }

    public bool IsFinished(SelfdriveContext context)
    {
        if (_startTime == 0)
        {
            return false;
        }
        else if ((TimeUtils.Now() - _startTime) > timeout)
        {
            return true;
        }
        else
        {
            return false; //FIXME detect if there's an obstacle within 3 feet of us
        }
    }
}