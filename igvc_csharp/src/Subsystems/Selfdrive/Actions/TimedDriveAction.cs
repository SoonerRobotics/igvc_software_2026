using igvc_csharp.Core;
using igvc_csharp.Core.Units;
using igvc_csharp.src.selfdrive.actions;
using igvc_csharp.Utils;

namespace igvc_csharp.src.Subsystems.selfdrive.actions;

public class TimedDriveAction(
    LinearVelocity forwardSpeed,
    LinearVelocity sidewaysSpeed,
    AngularVelocity turnSpeed,
    ulong timeoutMs
) : ISelfdriveAction
{
    private ulong _startTime;
    private bool _init;

    public bool IsInit() => _init;

    public void Init(SelfdriveContext context)
    {
        _startTime = TimeUtils.Now();
        _init = true;
    }

    public void Run(SelfdriveContext context)
    {
        if (BaseRobot.Instance?.State.MotionAllowed ?? false)
        {
            context.Canbus.MotorControl.SetVelocities(
                forwardSpeed.To(LinearVelocityUnit.MetersPerSecond),
                sidewaysSpeed.To(LinearVelocityUnit.MetersPerSecond),
                turnSpeed.To(AngularVelocityUnit.RadiansPerSecond));
        }
        else
        {
            context.Canbus.MotorControl.SetVelocities(0, 0, 0);
        }
    }

    public void End(SelfdriveContext context)
    {
        context.Canbus.MotorControl.SetVelocities(0, 0, 0);
    }

    public bool IsFinished(SelfdriveContext context)
    {
        if (!_init)
            return false;

        return (TimeUtils.Now() - _startTime) > timeoutMs;
    }
}