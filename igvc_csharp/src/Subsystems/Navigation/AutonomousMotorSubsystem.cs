using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Subsystems.Navigation;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Motor;

/// <summary>
/// Reads the planned path from NavigationSubsystem via PurePursuit and
/// drives the motors via MotorControlLayer.
///
/// Commands are only sent when:
///   - RobotMode == Autonomous
///   - MotionAllowed == true
///   - Estopped == false
///
/// When any of those conditions fail, a zero-velocity command is sent once
/// to ensure the robot comes to a stop, then the subsystem stays quiet
/// until conditions are met again.
/// </summary>
[Subsystem("MotorSubsystem", Disabled = false)]
public class MotorSubsystem(
    CanbusSubsystem canbus,
    NavigationSubsystem navigation
) : SubsystemBase
{
    // Pure Pursuit config — tune these to match your robot
    private const float RadiusStart = 0.7f;
    private const float RadiusMultiplier = 1.2f;
    private const float RadiusMax = 4.0f;

    // Velocity limits
    private const double ForwardSpeed = 0.5;
    private const double AngularAggression = 1.8;
    private const double MaxAngularSpeed = 0.8;

    // Minimum lookahead distance — below this the robot is considered "at goal"
    private const float AtGoalDistanceSq = 0.25f;

    private readonly PurePursuit _pursuit = new();

    // Tracks whether we've already sent the stop command after dropping out
    // of the active state, so we don't spam zeros every tick.
    private bool _sentStopOnExit = false;

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
        // When we leave the active window, send one immediate stop so the
        // robot doesn't coast until the next control loop tick.
        if (IsActive(old) && !IsActive(updated))
        {
            SendVelocities(0, 0, 0);
            _sentStopOnExit = true;
        }

        return Task.CompletedTask;
    }

    private async Task ControlLoop(CancellationToken token)
    {
        // 20 Hz — matches the Python resolver's 0.05 s timer
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(50));

        try
        {
            while (await timer.WaitForNextTickAsync(token))
            {
                var state = BaseRobot.Instance.State;

                if (!IsActive(state))
                {
                    // One zero has already been sent by OnRobotStateChanged;
                    // don't flood the bus.
                    if (!_sentStopOnExit)
                    {
                        SendVelocities(0, 0, 0);
                        _sentStopOnExit = true;
                    }
                    continue;
                }

                _sentStopOnExit = false;

                // Sync pursuit points from the latest plan
                if (navigation.LastLocalPath is { Count: > 0 } localPath)
                    _pursuit.SetPoints(localPath);

                if (_pursuit.Count == 0)
                {
                    Logger.LogDebug("No path available, holding position");
                    SendVelocities(0, 0, 0);
                    continue;
                }

                // Robot is always at the local-frame origin
                (float X, float Y)? lookahead = null;
                float radius = RadiusStart;
                while (lookahead is null && radius <= RadiusMax)
                {
                    lookahead = _pursuit.GetLookaheadPoint(0f, 0f, radius);
                    radius *= RadiusMultiplier;
                }

                if (lookahead is null)
                {
                    // Path exists but lookahead missed — back up gently (mirrors Python resolver)
                    Logger.LogDebug("No lookahead found, reversing");
                    SendVelocities(-0.4, 0, 0);
                    continue;
                }

                float distSq = lookahead.Value.X * lookahead.Value.X
                             + lookahead.Value.Y * lookahead.Value.Y;

                if (distSq <= AtGoalDistanceSq)
                {
                    Logger.LogDebug("At goal, holding position");
                    SendVelocities(0, 0, 0);
                    continue;
                }

                // Heading error in [-1, 1] (normalised by π)
                double angleToLookahead = Math.Atan2(lookahead.Value.Y, lookahead.Value.X);
                double error = NormaliseAngle(angleToLookahead) / Math.PI;

                // Speed tapers off sharply when pointing away from the target
                double forward = ForwardSpeed * Math.Pow(1.0 - Math.Abs(error), 5);
                double angular = Math.Clamp(
                    error * AngularAggression,
                    -MaxAngularSpeed,
                    MaxAngularSpeed);

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

    // ── Helpers ──────────────────────────────────────────────────────────────

    private static bool IsActive(RobotState state) =>
        state.Mode == RobotModeEnum.Autonomous
        && state.MotionAllowed
        && !state.Estopped;

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

    /// <summary>Wraps angle to [-π, π].</summary>
    private static double NormaliseAngle(double angle) =>
        (angle + Math.PI) % (2 * Math.PI) - Math.PI;
}