
using igvc_csharp.src.selfdrive.actions;
using igvc_csharp.src.subsystems.selfdrive;
using igvc_csharp.Utils;

namespace igvc_csharp.src.Subsystems.selfdrive.actions;

public class LaneChangeAction(ulong timeoutMs = 0) : ISelfdriveAction
{
    private ulong _startTime = 0;
    private bool _init = false;
    private SelfdriveContext? context;
    private SelfdriveLane? goalLane;

    public bool IsInit()
    {
        return _init;
    }

    public void Init(SelfdriveContext context)
    {
        _startTime = TimeUtils.Now();
        _init = true;
        this.context = context;

        if (context.CurrentLane == SelfdriveLane.Left)
        {
            goalLane = SelfdriveLane.Right;
        }
        else
        {
            goalLane = SelfdriveLane.Left;
        }
    }

    public void Run(SelfdriveContext context)
    {
        if (goalLane == SelfdriveLane.Left)
        {
            // strafe left
            context.canbus.MotorControl.SetVelocities(1, 0.5, 0); //FIXME make this configurable?
        }
        else
        {
            // strafe right
            context.canbus.MotorControl.SetVelocities(1, -0.5, 0); //FIXME make this configurable?
        }
    }

    public void End(SelfdriveContext context)
    {
        //TODO: do nothing??? Mat.Dispose()? set motors to 0?
    }

    public bool IsFinished(SelfdriveContext context)
    {
        if (_startTime == 0)
        {
            return false;
        }
        else if ((TimeUtils.Now() - _startTime) > timeoutMs && timeoutMs > 0)
        {
            return true;
        }

        return context.CurrentLane == goalLane;
    }
}