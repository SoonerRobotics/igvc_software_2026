using igvc_csharp.Core;
using igvc_csharp.Core.Units;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Utils;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems;

[Subsystem("ManualControlSubsystem", DependsOn=[typeof(ControllerSubsystem)])]
public class ManualControlSubsystem(ControllerSubsystem controller, CanbusSubsystem canbus) : SubsystemBase
{
    private DateTime? _dpadDepressedAt;
    private bool _dpadFlashed;

    private double _angularVelocity;
    private double _forwardVelocity;
    private double _sidewaysVelocity;
    
    public override Task Init(CancellationToken token)
    {
        ControllerHooks(token);
        _ = SendLoop(token);
        return Task.CompletedTask;
    }

    private async Task SendLoop(CancellationToken token)
    {
        using var timer = new PeriodicTimer(Configuration.DriveSubsystem.UpdateFrequency);
        while (await timer.WaitForNextTickAsync(token))
        {
            // Only send in manual mode
            if (Robot.Instance.State.Mode != RobotModeEnum.Manual)
            {
                continue;
            }
            
            canbus.MotorControl.SetVelocities(_forwardVelocity, _sidewaysVelocity, _angularVelocity);
        }
    }

    private static double ApplyDeadband(double value, double deadband = 0.05)
    {
        if (Math.Abs(value) < deadband)
        {
            return 0.0;
        }

        // Rescales such that output starts just past the deadband
        return (value - deadband * Math.Sign(value)) / (1.0 - deadband);
    }

    private void ControllerHooks(CancellationToken token)
    {
        // System Mode
        controller.Buttons.Menu.OnReleased += () =>
        {
            SetRobotMode(RobotModeEnum.Manual);
            canbus.SafetyLights.SetManual();
        };

        controller.Buttons.Xbox.OnReleased += () =>
        {
            SetRobotMode(RobotModeEnum.Autonomous);
            canbus.SafetyLights.SetAutonomous();
        };

        controller.Buttons.View.OnReleased += () =>
        {
            SetRobotMode(RobotModeEnum.Disabled);
            canbus.SafetyLights.SetDisabled();
        };
        
        // System Mission
        controller.Dpad.DpadRight.OnPressed += () =>
        {
            _dpadDepressedAt = DateTime.Now;
        };
        controller.Dpad.DpadRight.WhileHeld += () =>
        {
            if (_dpadDepressedAt == null || _dpadFlashed) return;
            
            if ((DateTime.Now - _dpadDepressedAt).Value.Milliseconds <= 1500)
            {
                return;
            }
            
            _dpadFlashed = true;
            canbus.SafetyLights.FlashTemporary(ColorUtils.Color.CadetBlue, token, length: 1200);
            Logger.LogDebug("Flashing DPad for mission switch");
        };
        controller.Dpad.DpadRight.OnReleased += () =>
        {
            if (_dpadDepressedAt == null) return;

            _dpadFlashed = false;
            if ((DateTime.Now - _dpadDepressedAt).Value.Milliseconds <= 3000)
            {
                _dpadDepressedAt = null;
                return;
            }

            _dpadDepressedAt = null;
            SetRobotMission(Robot.Instance.State.Mission == MissionEnum.Autonav
                ? MissionEnum.Selfdrive
                : MissionEnum.Autonav);
            Logger.LogDebug("Switching Mission: {Mission}", Robot.Instance.State.Mission);
        };
        
        // Rotation
        controller.Axes.LeftStick.OnChanged += (x, y) =>
        {
            // Convert to m/s
            _forwardVelocity = ApplyDeadband(y) * Configuration.DriveSubsystem.MaxForwardSpeed.ToMetersPerSecond();
            _sidewaysVelocity = ApplyDeadband(x) * Configuration.DriveSubsystem.MaxSidewaysSpeed.ToMetersPerSecond();

            // Invert if needed
            _forwardVelocity *= Configuration.DriveSubsystem.InvertForwardVelocity ? -1 : 1;
            _sidewaysVelocity *= Configuration.DriveSubsystem.InvertSidewaysVelocity ? -1 : 1;
        };

        // Drive
        controller.Axes.RightStick.OnChanged += (x, y) =>
        {
            // Convert to rad/s
            _angularVelocity = ApplyDeadband(x) * Configuration.DriveSubsystem.MaxAngularSpeed.To(AngularVelocityUnit.RadiansPerSecond);

            // Invert if needed
            _angularVelocity *= Configuration.DriveSubsystem.InvertAngularVelocity ? -1 : 1;
        };
    }
}