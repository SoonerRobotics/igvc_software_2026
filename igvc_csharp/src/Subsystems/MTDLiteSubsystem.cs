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
using Priority_Queue;


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
    private SimplePriorityQueue<(int, int), int> _openSet = new(); //TODO rename to like... "gridMap" or something?
    //FIXME no, _openSet is supposed to be a List<> I think...
    private double _km = 0;
    private int[,] _parents = new int[46, 46]; //FIXME make this size configurable
    private (int, int) _startPos;
    private (int, int) _goalPos;

    // D* Lite maintains an h-value, g-value, f -value
    // and parent pointer for every state s with similar meanings as
    // used by A* but it also maintain an rhs-value
    private int[,] _gCost = new int[46, 46]; //FIXME should these be ints? or like, shorts? bytes even?
    private int[,] _hCost = new int[46, 46];
    private int[,] _fCost = new int[46, 46];
    private int[,] _rhs = new int[46, 46];

    /*
        S denotes the finite set
        of all (blocked and unblocked) states, sstart ∈ S denotes the
        current state of the hunter and the start state of the search,
        and sgoal ∈ S denotes the current state of the target and the
        goal state of the search. c(s, s′) denotes the cost of a cost-
        minimal path from state s ∈ S to state s′ ∈ S. Succ(s) ⊆ S
        denotes the set of successors of state s ∈ S, and Pred(s) ⊆ S
        denotes the set of predecessors of state s ∈ S. 
    */

    private const int Infinity = int.MaxValue;

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        Initialize();

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

    private int CalculateKey((int, int) pt)
    {
        //k1 = min(self.g[s], self.rhs[s]) + heuristic(self.s_start, s) + self.k_m
        // k2 = min(self.g[s], self.rhs[s])
        // return (
        //     Math.Min(_gCost[pt.Item1, pt.Item2], _rhs[pt.Item1, pt.Item2]) + Heurisitic(pt, _goalPos) + _km, Math.Min(_gCost[pt.Item1, pt.Item2], _rhs[pt.Item1, pt.Item2])
        // );

        return -1;
    }

    private double Cost((int, int) p1, (int, int) p2)
    {
        // TODO: check if p1 or p2 are obstructed
        return Heurisitic(p1, p2);
    }

    private double Heurisitic((int, int) p1, (int, int) p2)
    {
        // just use regular distance formula
        // TODO: is this right?
        return Math.Sqrt(Math.Pow(p1.Item1 - p2.Item1, 2) + Math.Pow(p1.Item2 - p2.Item2, 2));
    }

    private void Initialize()
    {
        _openSet.Clear(); //FIXME is this right ???
        _km = 0;

        for (int y = 0; y < _fCost.GetLength(0); y++)
        {
            for (int x = 0; x < _fCost.GetLength(1); x++)
            {
                _rhs[x, y] = Infinity;
                _gCost[x, y] = Infinity;
                // _parents[x, y] = null;
            }
        }

        _startPos = new(); //TODO FIXME
        _goalPos = new(); //TODO FIXME

        _openSet.Enqueue(_startPos, CalculateKey(_startPos));
    }

    private void UpdateState((int, int) point)
    {
        if (_gCost[point.Item1, point.Item2] != _rhs[point.Item1, point.Item2])
        {
            if (_openSet.Contains(point))
            {

                _openSet.UpdatePriority(point, CalculateKey(point));
            }
            else
            {
                _openSet.Enqueue(point, CalculateKey(point));
            }
        }
        else if (_openSet.Contains(point))
        {
            //FIXME do something if we didn't actually delete it (i.e. if this returns false or if _ and __ are wrong)
            _openSet.Remove(point);
        }
    }

    private void ComputeCostMinimalPath()
    {
        // while (_openSet.Dequeue() < CalculateKey(_goalPos)
        //     || _rhs[_goalPos.Item1, _goalPos.Item2] > _gCost[_goalPos.Item1, _goalPos.Item2])
        // {
        //     var workingPt = _openSet.Dequeue();
        //     var newMinPriority = CalculateKey(workingPt);

        //     // if our lowest cost node has a lower priority now
        //     if (newMinPriority < workingMinPriority)
        //     {
        //         // then update it ??? FIXME?
        //         _openSet.UpdatePriority(workingPt, workingMinPriority);
        //     }
        //     // otherwise, if the G cost has increased
        //     else if (_gCost[workingPt.Item1, workingPt.Item2] > _rhs[workingPt.Item1, workingPt.Item2])
        //     {
        //         // update the g cost with RHS ???
        //         _gCost[workingPt.Item1, workingPt.Item2] = _rhs[workingPt.Item1, workingPt.Item2];

        //         _openSet.Remove(workingPt); //FIXME actually check if this succeeded

        //         foreach (var pt in Succ(workingPt))
        //         {
        //             if (pt != _startPos && _parents[pt] == workingPt)
        //             {
        //                 // _rhs[pt] = Math.Min(_gCost); // ??? line {39} FIXME
        //                 if (_rhs[pt] == Infinity)
        //                 {
        //                     _parents[pt] = null;
        //                 }
        //                 else
        //                 {
        //                     // _parents[pt] = argmin(); // ????? line {43} FIXME
        //                 }
        //             }

        //             UpdateState(pt);
        //         }
        //     }
        //     else
        //     {
        //         // otherwise this node isn't worth exploring ??? FIXME ???
        //         _gCost[workingPt.Item1, workingPt.Item2] = Infinity;
        //         //TODO foreach loop (line {36})
        //     }
        // }
    }

    /**
     * TODO FIXME
     */
    private Task CalculatePath(CancellationToken token)
    {
        while (_startPos != _goalPos)
        {
            var oldStart = _startPos;
            var oldGoal = _goalPos;

            ComputeCostMinimalPath();
            
            // no path exists
            if (_rhs[_goalPos.Item1, _goalPos.Item2] == Infinity)
            {
                return Task.CompletedTask;
                // return false;
            }

            //TODO: identify a path from sstart to sgoal using the parent pointers;
            // while (_currentPos != _goalPos && path.Contains(_goalPos) && no edge costs changed) {
            //     hunter follows path from sstart to sgoal;
            // }
            if (_startPos == _goalPos)
            {
                //FIXME what do we even do here ???
                return Task.CompletedTask;
            }
            // _startPos = the current state of the hunter;
            // _goalPos = the current state of the target;
            _km += Heurisitic(oldGoal, _goalPos);
            if (oldStart != _startPos)
            {
                // shift the map appropriately(which changes sstart and sgoal);
                //             for all directed edges(u, v) with changed edge costs

                // c_old = c(u, v);
                // update the edge cost c(u, v);
                // if (c_old > c(u, v))
                // {

                //     if (v != _startPos && _rhs[v] > (_gCost[u] + c(u, v)))
                //     {
                //         par(v) = u;
                //         rhs(v) = _gCost[u] + c(u, v);
                //         UpdateState(v);
                //     }
                // }
                // else
                // {
                //     if (v != _startPos && par(v) == u)
                //     {
                //         // _rhs[v] = mins′∈Pred(v)(g(s′) + c(s′, v));
                //         if (_rhs[v] == Infinity)
                //         {
                //             _par(v) = null;
                //         }
                //         else
                //         {
                //             _par(v) = arg mins′∈Pred(v)(_gCost[s′] + c(s′, v));
                //         }
                //         UpdateState(v);
                //     }
                // }
                // return true;
            }
        }

        return Task.CompletedTask;
    }
}