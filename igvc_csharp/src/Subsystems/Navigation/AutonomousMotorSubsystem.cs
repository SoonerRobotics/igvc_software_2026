using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Subsystems.Navigation;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Motor;

[Subsystem("MotorSubsystem", Disabled = false)]
public class MotorSubsystem(
    CanbusSubsystem canbus,
    NavigationSubsystem navigation
) : SubsystemBase
{
    private const float RadiusStart = 0.7f;
    private const float RadiusMultiplier = 1.2f;
    private const float RadiusMax = 4.0f;

    // Velocity limits
    [Config("autonomous.forward_speed")]
    public static double ForwardSpeed = 0.80;

    [Config("autonomous.angular_aggression")]
    public static double AngularAggression = 2.0;

    [Config("autonomous.max_angular_speed")]
    public static double MaxAngularSpeed = 1.8;

    public const float AtGoalDistanceSq = 0.1f;

    private readonly PurePursuit _pursuit = new();

    private int _reverseFrames = 0;

    public override Task Init(CancellationToken token)
    {
        _ = Task.Factory.StartNew(
            () => ControlLoop(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        SetOperatingState(SubsystemState.Operating);
        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        if (IsAutonomous(old) && !IsAutonomous(updated))
        {
            SendVelocities(0, 0, 0);
            return Task.CompletedTask;
        }

        if (!IsAutonomous(old) && IsAutonomous(updated))
        {
            _reverseFrames = 0;
            _pursuit.SetPoints(new List<(float, float)>());
            return Task.CompletedTask;
        }

        if (!updated.MotionAllowed && updated.Mode == RobotModeEnum.Autonomous)
        {
            SendVelocities(0, 0, 0);
            canbus.SafetyLights.SetAutoDisabled();
            return Task.CompletedTask;
        }

        if (updated.MotionAllowed && updated.Mode == RobotModeEnum.Autonomous)
        {
            canbus.SafetyLights.SetAutoEnabled();
        }

        return Task.CompletedTask;
    }

    private async Task ControlLoop(CancellationToken token)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(20));

        try
        {
            while (await timer.WaitForNextTickAsync(token))
            {
                var state = BaseRobot.Instance.State;

                if (!IsAutonomous(state))
                {
                    SendVelocities(0, 0, 0);
                    continue;
                }

                if (_reverseFrames > 0)
                {
                    SendVelocities(-0.4, 0, 0.1);
                    _reverseFrames--;
                    continue;
                }

                if (navigation.LastLocalPath is { Count: > 0 } localPath)
                {
                    _pursuit.SetPoints(localPath);
                }

                if (_pursuit.Count == 0)
                {
                    SendVelocities(0, 0, 0);
                    continue;
                }

                (float X, float Y)? lookahead = null;
                float radius = RadiusStart;
                while (lookahead is null && radius <= RadiusMax)
                {
                    lookahead = _pursuit.GetLookaheadPoint(0f, 0f, radius);
                    radius *= RadiusMultiplier;
                }

                if (lookahead is null)
                {
                    SendVelocities(-0.4, 0, 0);
                    continue;
                }

                float distSq = lookahead.Value.X * lookahead.Value.X + lookahead.Value.Y * lookahead.Value.Y;
                if (distSq <= AtGoalDistanceSq)
                {
                    SendVelocities(0, 0, 0);
                    _reverseFrames = 15;
                    continue;
                }

                double angleToLookahead = Math.Atan2(lookahead.Value.Y, lookahead.Value.X);
                double error = NormaliseAngle(angleToLookahead) / Math.PI;
                if (Math.Abs(error) < 0.02)
                {
                    error = 0;
                }

                double forward = ForwardSpeed * Math.Max(0.25, Math.Pow(1.0 - Math.Abs(error), 2));
                double angular = Math.Clamp(
                    error * AngularAggression,
                    -MaxAngularSpeed,
                    MaxAngularSpeed
                );
                SendVelocities(forward, 0, angular);
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Motor control loop crashed");
            SendVelocities(0, 0, 0);
        }
    }

    private static bool IsAutonomous(RobotState state) => state.Mode == RobotModeEnum.Autonomous && state.MotionAllowed;

    private void SendVelocities(double forward, double sideways, double angular)
    {
        try
        {
            canbus.MotorControl.SetVelocities(forward, sideways, angular);
        }
        catch (Exception ex)
        {
            Logger.LogWarning(ex, "Failed to send motor command");
        }
    }

    private static double NormaliseAngle(double angle) => (angle + Math.PI) % (2 * Math.PI) - Math.PI;
}