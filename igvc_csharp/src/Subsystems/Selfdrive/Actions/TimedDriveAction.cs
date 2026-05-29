
using igvc_csharp.Core;
using igvc_csharp.src.selfdrive.actions;
using igvc_csharp.Utils;

namespace igvc_csharp.src.Subsystems.selfdrive.actions;

public class TimedDriveAction(double forwardSpeed, double sidewaysSpeed, double turnSpeed, ulong timeoutMs) : ISelfdriveAction
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
        if (BaseRobot.Instance?.State.MotionAllowed ?? false)
        {
            context.canbus.MotorControl.SetVelocities(forwardSpeed, sidewaysSpeed, turnSpeed);
        }
        else
        {
            //TODO: write 0 to motors?
        }
    }

    public void End(SelfdriveContext context)
    {
        //TODO: do nothing??? Mat.Dispose()?

        //TODO: set motors to 0?
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