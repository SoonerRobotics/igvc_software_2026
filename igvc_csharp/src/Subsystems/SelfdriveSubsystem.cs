using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;

[Subsystem("SelfdriveSubsystem", Disabled = true)]
public class SelfdriveSubsystem(
    CanbusSubsystem? canbus
) : SubsystemBase
{
    private double _smoothForward = 0;
    private double _smoothSideways = 0;
    private double _smoothAngular = 0;

    private const double Alpha = 0.13;
    private const double AlphaAngular = 0.08;

    private int _missedFrames = 0;
    private const int MaxMissedFrames = 12;

    public override Task Init(CancellationToken token)
    {
        Subscribe<YoloDetectionEvent>(OnDetectionEvent, token);
        return Task.CompletedTask;
    }

    private void FollowObject(YoloDetectionEvent e)
    {
        if (BaseRobot.Instance?.State.Mode != RobotModeEnum.Autonomous)
            return;

        double targetForward, targetSideways, targetAngular;

        if (e.x == -1 && e.y == -1 && e.z == -1)
        {
            _missedFrames++;
            if (_missedFrames >= MaxMissedFrames)
            {
                targetForward = 0;
                targetSideways = 0;
                targetAngular = 0;
            }
            else
            {
                targetForward = _smoothForward;
                targetSideways = _smoothSideways;
                targetAngular = _smoothAngular;
            }
        }
        else
        {
            _missedFrames = 0;

            float forwardDist = -e.z;
            float lateralDist = e.x;
            double angleToPerson = Math.Atan2(lateralDist, forwardDist);

            const double targetDistance = 1.5;
            const double forwardGain = 0.4;
            double rawForward = forwardGain * (forwardDist - targetDistance);
            targetForward = Math.Abs(rawForward) < 0.05 ? 0 : Math.Clamp(rawForward, -0.5, 0.8);

            const double lateralGain = 0.1;
            targetSideways = Math.Abs(lateralDist) < 0.1 ? 0
                : Math.Clamp(-lateralGain * lateralDist, -0.3, 0.3);

            const double angularGain = 0.4;
            const double angularDeadband = 0.08;
            targetAngular = Math.Abs(angleToPerson) < angularDeadband ? 0
                : Math.Clamp(-angularGain * angleToPerson, -1.0, 1.0);

            // Suppress lateral movement when turning significantly
            if (Math.Abs(targetAngular) > 0.2)
                targetSideways = 0;
        }

        _smoothForward += Alpha * (targetForward - _smoothForward);
        _smoothSideways += Alpha * (targetSideways - _smoothSideways);
        _smoothAngular += AlphaAngular * (targetAngular - _smoothAngular);

        canbus?.MotorControl.SetVelocities(_smoothForward, _smoothSideways, _smoothAngular);
    }

    public Task OnDetectionEvent(YoloDetectionEvent e, CancellationToken token)
    {
        if (e.label == "person")
            FollowObject(e);

        return Task.CompletedTask;
    }
}