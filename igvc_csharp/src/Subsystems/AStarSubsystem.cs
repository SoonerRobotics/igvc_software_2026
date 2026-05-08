using System.Diagnostics;
using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using igvc_csharp.Core.Units;
using igvc_csharp.Subsystems.Tools;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Core.Hardware;
using OpenCvSharp;
using AStarConfig = igvc_csharp.Configuration.AStarSubsystem;
using igvc_csharp.src.Utils;


namespace igvc_csharp.scr.Subsystems;

[Subsystem("AStarSubsystem", Disabled = false)]
public class AStarSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    SCR_Point _goalPoint;
    List<SCR_Point> _path = [];

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        //TODO: subscribers / publishers

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    public void Reset()
    {
        _last_path = null;
        _position = null;
        _config_space = null;
        _cost_map = null;
        _best_pos = null;

        SetOperatingState(SubsystemState.Ready);
    } 

    public override Task OnRobotModeChanged(RobotModeEnum old, RobotModeEnum current)
    {
        // on an actual mode change FIXME we might want to reset for mob start/stop too?
        if (old != current)
        {
            if (current == RobotModeEnum.Autonomous)
            {
                Reset();
            }
        }

        //TODO: are we supposed to control the safety lights as well? or only for debug information?

        return Task.CompletedTask;
    }

    public void FindPath()
    {
        
    }
}