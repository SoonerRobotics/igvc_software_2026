using igvc_csharp.Core;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Utils;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems;

[Subsystem("ManualControlSubsystem", DependsOn=[typeof(ControllerSubsystem)])]
public class ManualControlSubsystem(ControllerSubsystem controller, CanbusSubsystem canbus) : SubsystemBase
{
    private DateTime? _dpadDepressedAt;
    private bool _dpadFlashed;
    
    public override Task Init(CancellationToken token)
    {
        ControllerHooks(token);

        return Task.CompletedTask;
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
    }
}