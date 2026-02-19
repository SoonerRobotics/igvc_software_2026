using igvc_csharp.Core;
using igvc_csharp.Subsystems.Hardware;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems;

[Subsystem("ManualControlSubsystem", DependsOn=[typeof(ControllerSubsystem)])]
public class ManualControlSubsystem(ControllerSubsystem controller, CanbusSubsystem canbus) : SubsystemBase
{
    private DateTime? _dpadDepressedAt;
    private bool _dpadFlashed;
    
    public override Task Init(CancellationToken token)
    {
        ControllerHooks();

        return Task.CompletedTask;
    }

    private void ControllerHooks()
    {
        // System Mode
        controller.Buttons.Menu.OnReleased += () =>
        {
            // TODO: Convert these to functions maybe?
            Robot.Instance.State.Mode = RobotModeEnum.Manual;
        };

        controller.Buttons.Xbox.OnReleased += () =>
        {
            Robot.Instance.State.Mode = RobotModeEnum.Autonomous;
        };

        controller.Buttons.View.OnReleased += () =>
        {
            Robot.Instance.State.Mode = RobotModeEnum.Disabled;
        };
        
        // System Mission
        controller.Dpad.DpadRight.OnPressed += () =>
        {
            _dpadDepressedAt = DateTime.Now;
        };
        controller.Dpad.DpadRight.WhileHeld += () =>
        {
            if (_dpadDepressedAt == null || _dpadFlashed) return;
            
            if ((DateTime.Now - _dpadDepressedAt).Value.Seconds <= 2)
            {
                return;
            }
            
            _dpadFlashed = true;
            // TODO: Flash lights and vibrate remote
            canbus.SafetyLights.Flash();
            Logger.LogDebug("Flashing DPad for mission switch");
        };
        controller.Dpad.DpadRight.OnReleased += () =>
        {
            if (_dpadDepressedAt == null) return;

            _dpadFlashed = false;
            if ((DateTime.Now - _dpadDepressedAt).Value.Seconds <= 2)
            {
                _dpadDepressedAt = null;
                return;
            }

            _dpadDepressedAt = null;
            Robot.Instance.State.Mission = Robot.Instance.State.Mission == MissionEnum.Autonav
                ? MissionEnum.Selfdrive
                : MissionEnum.Autonav;
            Logger.LogDebug("Switching Mission: {Mission}", Robot.Instance.State.Mission);
        };
    }
}