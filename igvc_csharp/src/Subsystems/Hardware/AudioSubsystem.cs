using System.Collections.Concurrent;
using System.Reflection.Metadata;
using System.Runtime.InteropServices;
using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Messages;
using igvc_csharp.Subsystems;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Subsystems.Simulator;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using System.Diagnostics;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Hardware;


//TODO: make this subsystem like, configurable and everything
[Subsystem("AudioSubsystem", DependsOn = [typeof(ChronosSubsystem)])]
public class AudioSubsystem(
    SimulatorSubsystem? simulatorSubsystem,
    ChronosSubsystem chronos
) : SubsystemBase
{
    private bool _isPlayingSound = false;
    private string _currentPlayingSound = ""; //FIXME not sure if we need/want this
    private Process? _process;

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        SubscribeMessage<AudibleFeedback>(
            MessageType.AudibleFeedback,
            OnMessageReceived,
            token
        );

        //TODO: subscribe to Waypoint messages as well? if that code does work and gets merged...

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        //Logger.LogDebug("YES WE ARE GETTING CALLED!!!!");
        
        // check for a change in mode
        if (old.Mode != updated.Mode)
        {
            if (updated.Mode == RobotModeEnum.Autonomous)
            {
                if (BaseRobot.Instance.State.Mission == MissionEnum.Autonav)
                {
                    PlaySound("autonav-mode.mp3");
                }
                else
                {
                    PlaySound("self-drive.mp3");
                }
            }
            else if (updated.Mode == RobotModeEnum.Manual)
            {
                PlaySound("self-drive2.mp3");
            }
            else if (updated.Mode == RobotModeEnum.Disabled)
            {
                //TODO I don't think we need a sound for this (?) but it's here...
            }
        }
        // no change in robot mode, so it must be something else that changed (i.e. mission or mobility)
        else
        {
            if (BaseRobot.Instance.State.MotionAllowed)
            {
                PlaySound("mobility-enable.mp3");
            }
            else
            {
                PlaySound("mobility-stop.mp3");
            }
        }

        return Task.CompletedTask;
    }

    private Task OnMessageReceived(AudibleFeedback msg, CancellationToken token)
    {
        if (msg.StopAllSounds)
        {
            StopAllSounds(token);
        }

        if (msg.Filename != "")
        {
            PlaySound(msg.Filename);
        }

        return Task.CompletedTask;
    }

    private Task PlaySound(string filename)
    {
        SetOperatingState(SubsystemState.Operating);

        string relativeFilename = FileUtils.GetFileRelativeToRoot("resources/" + filename);

        Logger.LogDebug("Playing sound: " + relativeFilename);

        using (Process playSound = new())
        {
            playSound.StartInfo.UseShellExecute = true;
            playSound.StartInfo.FileName = "ffplay"; //TODO FIXME
            playSound.StartInfo.CreateNoWindow = true; // no GUI on the headless NUC
            playSound.StartInfo.ErrorDialog = false;
            playSound.StartInfo.Arguments = " -nodisp -volume 100 -autoexit " + relativeFilename; //FIXME any other ffplay arguments we need to pass

            // I'm pretty sure this is non-blocking so we should keep track of it
            // there's a .WaitForExit() and a Kill() and a HasExited we can use and check
            playSound.Start();
        }

        return Task.CompletedTask;
    }

    private Task StopAllSounds(CancellationToken token)
    {
        _process.Kill();
        _isPlayingSound = false;

        // SetOperatingState(SubsystemState.Idle);

        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        SetOperatingState(SubsystemState.ShuttingDown);

        _process.Kill();
        _isPlayingSound = false;

        SetOperatingState(SubsystemState.Shutdown);

        return Task.CompletedTask;
    }
}
