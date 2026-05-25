using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;

[Subsystem("SelfdriveSubsystem")]
public class SelfdriveSubsystem(
    CanbusSubsystem? canbus
) : SubsystemBase
{
    public override Task Init(CancellationToken token)
    {
        Subscribe<YoloDetectionEvent>(OnDetectionEvent, token);

        return Task.CompletedTask;
    }

    private double DistanceToObject(YoloDetectionEvent e)
    {
        // The detection event has a x, y, z in world space calculated via a depth camera
        // We can use this to calculate the distance to the object
        return Math.Sqrt(e.x * e.x + e.y * e.y + e.z * e.z);
    }

    private double AngleToObject(YoloDetectionEvent e)
    {
        // The detection event has a x, y, z in world space calculated via a depth camera
        // We can use this to calculate the angle to the object
        return Math.Atan2(e.y, e.x);
    }

    private void FollowObject(YoloDetectionEvent e)
    {
        if (e.x == -1 && e.y == -1 && e.z == -1)
        {
            canbus?.MotorControl.SetVelocities(0, 0, 0);
            return;
        }

        float forwardDist = -e.z;
        float lateralDist = e.x;
        double angleToPerson = Math.Atan2(lateralDist, forwardDist);

        const double targetDistance = 1.5;
        const double forwardGain = 0.4;
        double distanceError = forwardDist - targetDistance;
        double forwardVelocity = forwardGain * distanceError;

        const double lateralGain = 0.1;
        double sidewaysVelocity = -lateralGain * lateralDist;

        const double angularGain = 0.8;
        double angularVelocity = -angularGain * angleToPerson;

        forwardVelocity = Math.Clamp(forwardVelocity, -0.5, 0.8);
        sidewaysVelocity = Math.Clamp(sidewaysVelocity, -0.3, 0.3);
        angularVelocity = Math.Clamp(angularVelocity, -1.0, 1.0);

        canbus?.MotorControl.SetVelocities(forwardVelocity, sidewaysVelocity, angularVelocity);
    }

    public Task OnDetectionEvent(YoloDetectionEvent e, CancellationToken token)
    {
        if (e.label == "person")
        {
            FollowObject(e);
        }

        return Task.CompletedTask;
    }
}