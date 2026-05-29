using igvc_csharp.Core;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using System.Diagnostics;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Hardware;


//TODO: make this subsystem like, configurable and everything
[Subsystem("AudioSubsystem")]
public class AudioSubsystem(ChronosSubsystem chronos) : SubsystemBase
{
    private Process? _process;

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        

        SubscribeMessage<AudibleFeedback>(
            MessageType.AudibleFeedback,
            OnMessageReceived,
            token
        );

        // set the default audio output to be the bluetooth speaker (Soundcore 2)
        using (Process setDefaultSink = new())
        {
            setDefaultSink.StartInfo.UseShellExecute = true;
            setDefaultSink.StartInfo.FileName = "pactl";
            setDefaultSink.StartInfo.CreateNoWindow = true;
            setDefaultSink.StartInfo.ErrorDialog = false;
            setDefaultSink.StartInfo.Arguments = " set-default-sink bluez_sink.8C_85_80_BC_73_39.handsfree_head_unit";
            setDefaultSink.Start();
        }

        // set the volume to be louder
        using (Process setVolume = new())
        {
            setVolume.StartInfo.UseShellExecute = true;
            setVolume.StartInfo.FileName = "ffplay";
            setVolume.StartInfo.CreateNoWindow = true;
            setVolume.StartInfo.ErrorDialog = false;
            setVolume.StartInfo.Arguments = " @DEFAULT_SINK@ 150%";
            setVolume.Start();
        }

        // play robot starting sound on robot code startup
        _ = Task.Factory.StartNew(
            () => PlaySound("robot-code-start.mp3"),
            token,
            TaskCreationOptions.None,
            TaskScheduler.Default
        );
        
        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        // check for a change in mode
        if (old.Mode != updated.Mode)
        {
            switch (updated.Mode)
            {
                case RobotModeEnum.Autonomous:
                    if (updated.Mission == MissionEnum.Autonav)
                    {
                        PlaySound("autonav-mode.mp3");
                    }
                    else
                    {
                        PlaySound("self-drive.mp3");
                    }
                    break;
                case RobotModeEnum.Manual:
                    PlaySound("self-drive2.mp3");
                    break;
                case RobotModeEnum.Disabled:
                    //TODO I don't think we need a sound for this (?) but it's here...
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }
        // check for mission change
        else if (old.Mission != updated.Mission)
        {
            switch (updated.Mission)
            {
                case MissionEnum.Autonav:
                    PlaySound("autonav-mode.mp3");
                    break;
                case MissionEnum.Selfdrive:
                    PlaySound("self-drive.mp3"); //FIXME
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }
        // no change in robot mode, so it must be something else that changed (i.e. mission or mobility)
        else
        {
            //FIXME do we even want to play these? I guess if nothing else has changed, for when mob-start is pressed on the e-stop remote,
            // but otherwise it's just going to overlap other sounds, no?
            if (!old.MotionAllowed && updated.MotionAllowed)
            {
                PlaySound("mobility-enable.mp3");
            }
            else if (old.MotionAllowed && !updated.MotionAllowed)
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

        string relativeFilename = FileUtils.GetFileRelativeToRoot("resources/audio/" + filename);

        Logger.LogDebug("Playing sound: " + relativeFilename);

        using (Process playSound = new())
        {
            playSound.StartInfo.UseShellExecute = true;
            playSound.StartInfo.FileName = "ffplay"; //TODO FIXME
            playSound.StartInfo.CreateNoWindow = true; // no GUI on the headless NUC
            playSound.StartInfo.ErrorDialog = false;
            playSound.StartInfo.Arguments = " -nodisp -volume 150 -autoexit -loglevel 8 " + relativeFilename; //FIXME any other ffplay arguments we need to pass

            // I'm pretty sure this is non-blocking so we should keep track of it
            // there's a .WaitForExit() and a Kill() and a HasExited we can use and check
            playSound.Start();
        }

        return Task.CompletedTask;
    }

    private Task StopAllSounds(CancellationToken token)
    {
        _process?.Kill();

        SetOperatingState(SubsystemState.Idle);

        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        SetOperatingState(SubsystemState.ShuttingDown);

        _process?.Kill();

        SetOperatingState(SubsystemState.Shutdown);

        return Task.CompletedTask;
    }
}
