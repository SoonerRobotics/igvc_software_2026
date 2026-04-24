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

namespace igvc_csharp.Subsystems.Hardware;


//TODO: config
[Subsystem("CanbusSubsystem", DependsOn = [
    typeof(ChronosSubsystem)
])]
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

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    private Task OnMessageReceived(AudibleFeedback msg, CancellationToken token)
    {
        if (msg.stop_all_sounds)
        {
            StopAllSounds(token);
        }

        if (msg.filename != "")
        {
            PlaySound(msg.filename, token);
        }

        return Task.CompletedTask;
    }

    private Task PlaySound(string filename, CancellationToken token)
    {
        SetOperatingState(SubsystemState.Operating);

        using (Process playSound = new())
        {
            playSound.StartInfo.UseShellExecute = true;
            playSound.StartInfo.FileName = "ffplay"; //TODO FIXME
            playSound.StartInfo.CreateNoWindow = true;
            playSound.StartInfo.ErrorDialog = false;
            playSound.StartInfo.Arguments = " -nodisp -volume 100 -autoexit " + filename; //TODO FIXME

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