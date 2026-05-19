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
using System.Security.Cryptography;
using Microsoft.VisualBasic;
using System.Security.Cryptography.X509Certificates;


namespace igvc_csharp.src.Subsystems;
/// <summary>
/// An implementation of Moving Target D* Lite
/// See the paper here: https://dlnext.acm.org/doi/pdf/10.5555/1838206.1838216
/// And the python implementation this is based on here: https://github.com/Sollimann/Dstar-lite-pathplanner
/// And the wiki page: TODO wiki link
/// </summary>
/// <param name="canbus"></param>
[Subsystem("Moving Target D* Lite Subsystem", Disabled = true)]
public class MTDLiteSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    //FIXME make this configurable, but this is basically the width and height of an autonav course in meters
    private PriorityQueue<(int, int), int> _openSet = new(); //TODO rename to like... "gridMap" or something?
    //FIXME no, _openSet is supposed to be a List<> I think...
    private int _km = 0;
    private int[46][46] _rhs = new();
    private int[46][46] _par = null;
    private (int, int) _startPos;
    private (int, int) _goalPos;

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        Initialize();

        // subscribers
        SubscribeImage(
            "combined_view",
            OnDebugImageReceived,
            token
        );

        SubscribeImage(
            "combined_filtered",
            OnMaskReceived,
            token
        );

        SubscribeMessage<VectornavReport>(
            MessageType.VectorNav,
            OnPositionReceived,
            token
        );

        SubscribeMessage<Waypoint>(
            MessageType.Waypoint,
            OnWaypointReceived,
            token
        );

        //TODO FIXME
        _ = Task.Factory.StartNew(
            () => CalculatePath(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        SetOperatingState(SubsystemState.Ready);
        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        //TODO

        return Task.CompletedTask;
    }

    private Task OnDebugImageReceived(ImageFrame frame, CancellationToken token)
    {
        _debugFrameChannel.Writer.TryWrite(frame);

        return Task.CompletedTask;
    }

    private Task OnMaskReceived(ImageFrame frame, CancellationToken token)
    {
        _maskFrameChannel.Writer.TryWrite(frame);

        SetOperatingState(SubsystemState.Operating);

        return Task.CompletedTask;
    }

    private Task OnPositionReceived(VectornavReport msg, CancellationToken token)
    {
        _position = msg;

        return Task.CompletedTask;
    }

    private Task OnWaypointReceived(Waypoint msg, CancellationToken token)
    {
        _goalPoint = new(msg.Latitude, msg.Longitude);

        return Task.CompletedTask;
    }

    // === actual D* Lite functions ===
    private int CalculateKey(SCR_Point pt)
    {
        return (
            Math.Min(_gGrid[pt], _rhs[pt]) + h(pt, _goal) + kM, Math.Min(_gGrid[pt], _rhs[pt])
        );
    }

    private void Initialize()
    {
        _openSet.Initialize(); // ??? fill with 0s?
        _km = 0;

        for (int y = 0; y < _openSet.Length; y++)
        {
            for (int x = 0; x < _openSet[y].Length; x++)
            {
                _rhs[y][x] = Infinity;
                _gCost[y][x] = Infinity;
                _par[y][x] = null;
            }
        }

        _startPos = new(); //TODO FIXME
        _goalPos = new(); //TODO FIXME

        //FIXME is openset supposed to be a priorityqueue or something?
        _openSet.Insert(_startPos, CalculateKey(_startPos));
    }

    private void UpdateState((int, int) point)
    {
        if (G(point) != _rhs[point])
        {
            if (_openSet.Contains(point))
            {

                _openSet.Update(point, CalculateKey(point));
            }
            else
            {
                _openSet.Enqueue(point, CalculateKey(point));
            }
        }
        else if (_openSet.Contains(point))
        {
            //FIXME do something if we didn't actually delete it (i.e. if this returns false or if _ and __ are wrong)
            _openSet.Remove(point, out _, out __);
        }
    }

    private void ComputeCostMinimalPath()
    {
        while (_openSet.TryPeek(out topKey, out topPriority) < CalculateKey(_goalPos) || _rhs(_goalPos) > G(_goalPos))
        {
            (var workingPt, var workingMinPriority) = _openSet.Dequeue();
            var newMinPriority = CalculateKey(workingPt);

            if (newMinPriority < workingMinPriority)
            {
                _openSet.Update(workingPt, workingMinPriority);
            }
            else if (G(workingPt) > _rhs[workingPt])
            {
                G(workingPt) = _rhs[workingPt];
                _openSet.Remove(workingPt, out _, out __); //FIXME actually check if this succeeded

                //TODO foreach loop (line {30})
            }
            else
            {
                G(workingPt) = Infinity;
                //TODO foreach loop (line {36})
            }
        }
    }

    /**
     * TODO FIXME
     */
    private async Task CalculatePath(CancellationToken token)
    {
        while (sstart  = sgoal)
soldstart:= sstart;
    soldgoal:= sgoal;
        ComputeCostMinimalPath();
        if (rhs(sgoal) = ∞) /*no path exists*/
return false;
        identify a path from sstart to sgoal using the parent pointers;
        while (target not caught AND target on path from sstart to sgoal
 AND no edge costs changed)
hunter follows path from sstart to sgoal;
        if hunter caught target
return true;
    sstart:= the current state of the hunter;
    sgoal:= the current state of the target;
    km:= km + h(soldgoal, sgoal);
        if (soldstart  = sstart)
shift the map appropriately(which changes sstart and sgoal);
        for all directed edges(u, v) with changed edge costs
        cold := c(u, v);
        update the edge cost c(u, v);
if (cold > c(u, v))
            if (v  = sstart AND rhs(v) > g(u) + c(u, v))
par(v) := u;
        rhs(v) := g(u) + c(u, v);
        UpdateState(v);
else
            if (v  = sstart AND par(v) = u)
rhs(v) := mins′∈Pred(v)(g(s′) + c(s′, v));
        if (rhs(v) = ∞)
par(v) := NULL;
else
            par(v) := arg mins′∈Pred(v)(g(s′) + c(s′, v));
        UpdateState(v);
        return true
    }
}