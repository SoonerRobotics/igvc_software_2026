using igvc_csharp.Core;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using igvc_csharp.Core.Units;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Core.Hardware;
using OpenCvSharp;
using AStarConfig = igvc_csharp.Configuration.AStarSubsystem;
using igvc_csharp.src.Utils;


namespace igvc_csharp.scr.Subsystems;

[Subsystem("AStarSubsystem", Disabled = false)]
public class AStarSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    private SCR_Point _goalPoint;
    private List<SCR_Point> _path = [];
    private SCR_Point _robotStartPoint = new(AStarConfig.ConfigSpaceWidth / 2, AStarConfig.ConfigSpaceHeight - 2);
    private LatLng? _waypoint;
    private LatLng? _position;
    private double _heading = 0;
    private uint[] _configSpace = [];

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        // subscribers
        SubscribeMessage<VectorNavReport>(
            MessageType.Gps,
            OnPositionReceived,
            token
        );

        SubscribeMessage<Waypoint>(
            MessageType.Waypoint,
            OnWaypointReceived,
            token
        );

        SubscribeMessage<ConfigSpace>(
            MessageType.ConfigSpace,
            OnConfigSpaceReceived,
            token
        );

        // publishers FIXME shouldn't this just run whenever we get a config space? and not on its own?
        _ = Task.Factory.StartNew(
            () => FindPath(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    private void Reset()
    {
        SetOperatingState(SubsystemState.Starting);

        _goalPoint = new();
        _path = [];
        _robotStartPoint = new(AStarConfig.ConfigSpaceWidth / 2, AStarConfig.ConfigSpaceHeight - 2);
        _waypoint = null;
        _position = null;
        _heading = 0;
        _configSpace = [];

        SetOperatingState(SubsystemState.Ready);
    }

    // gets the index of a 2D point in the 1D config space array
    private static int To1DArry(SCR_Point pt)
    {
        return (pt.Y * AStarConfig.ConfigSpaceWidth) + pt.X;
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

    private Task OnPositionReceived(VectorNavReport msg, CancellationToken token)
    {
        _position = new LatLng(msg.Latitude, msg.Longitude);

        _heading = msg.Yaw; //FIXME this doesn't get the right global heading right? we might have to store a _lastPosition or something

        return Task.CompletedTask;
    }

    private Task OnWaypointReceived(Waypoint msg, CancellationToken token)
    {
        _goalPoint = new(msg.Latitude, msg.Longitude);

        return Task.CompletedTask;
    }

    private Task OnConfigSpaceReceived(ConfigSpace msg, CancellationToken token)
    {
        //TODO FIXME this may need to be a deep copy or something?
        _configSpace = msg.GetDataArray();

        return Task.CompletedTask;
    }

    private Task FindGoalPoint(CancellationToken token)
    {
        // self.performance.start("Smellification")

        var workingGoalPoint = new SCR_Point(_robotStartPoint.X, _robotStartPoint.Y);
        double workingCost = -1000;

        HashSet<SCR_Point> frontier = [_robotStartPoint]; //FIXME we should try and re-use this for the A* or something.
        HashSet<SCR_Point> explored = [];

        //FIXME
        if (AStarConfig.UseOnlyWaypoints)
        {
            _configSpace = []; //TODO FIXME
        }

        int depth = 0;
        //FIXME add cancellation token check?
        while (depth < AStarConfig.SmellyMaxDepth && frontier.Count > 0)
        {
            var frontierCopy = frontier; //TODO make this a deep copy
            foreach (var point in frontierCopy)
            {
                double cost = (AStarConfig.ConfigSpaceHeight - point.Y) * AStarConfig.SmellyDistanceWeight + (depth * AStarConfig.SmellyDepthWeight);

                //FIXME better null check or something
                if (_goalPoint != null)
                {
                    var heading_err_to_gps = GetAngleDifference(_heading, GeoUtils.EstimateHeading(_position, _waypoint));
                    cost -= Math.Max(heading_err_to_gps, 10);
                }

                if (cost > workingCost)
                {
                    workingCost = cost;
                    workingGoalPoint = point;
                }

                frontier.Remove(point);
                explored.Add(point);

                //REALLY BIG FIXME: these !explored.Contains(point) need to be like, !explored.Contains(point.X + 1) typa thing y'know what I'm sayin?

                if (point.Y > 0 && _configSpace[To1DArry(point)] < 50 && !explored.Contains(point))
                {
                    // we're in image coordinates, so negative Y means upwards in the image, means forwards for the robot
                    frontier.Add(new SCR_Point(point.X, point.Y - 1));
                }

                if (point.X < (AStarConfig.ConfigSpaceWidth - 1) && _configSpace[To1DArry(point)] < 50 && !explored.Contains(point))
                {
                    frontier.Add(new SCR_Point(point.X + 1, point.Y));
                }

                if (point.X > 0 && _configSpace[To1DArry(point)] < 50 && !explored.Contains(point))
                {
                    frontier.Add(new SCR_Point(point.X - 1, point.Y));
                }
            }

            depth += 1;
        }

        // self.costMap = grid_data
        // self.performance.end("Smellification")

        _goalPoint = workingGoalPoint;

        return Task.CompletedTask;
    }

    //FIXME actually use cancellation token in loops n stuff
    private Task FindPath(CancellationToken token)
    {
        if (BaseRobot.Instance.State.Mode != RobotModeEnum.Autonomous || BaseRobot.Instance.State.Mission != MissionEnum.Autonav)
        {
            return Task.CompletedTask;
        }

        SetOperatingState(SubsystemState.Operating);

        FindGoalPoint(token);

        int[] lookedAt = new int[AStarConfig.ConfigSpaceWidth * AStarConfig.ConfigSpaceHeight];
        HashSet<SCR_Point> open_set = [_robotStartPoint];
        List<SCR_Point> path = [];
        List<(int, int, double)> search_dirs = [];
        SCR_Point currentPoint;

        // self.performance.start("A*")

        for (int x = -1; x < 2; x += 2)
        {
            for (int y = -1; y < 2; y += 2)
            {
                search_dirs.Add((x, y, Math.Sqrt((x * x) + (y * y))));
            }
        }

        double H(SCR_Point pt)
        {
            return pt.Dist(_goalPoint);
        }

        double D(SCR_Point pt)
        {
            return 0; //TODO FIXME what is this function even supposed to do? account for corners?
        }

        Dictionary<SCR_Point, int> gScore = new()
        {
            { _robotStartPoint, 0 } // initial point has score 0
        };

        int GetG(SCR_Point pt)
        {
            if (gScore.ContainsKey(pt))
            {
                return gScore[pt];
            }
            else
            {
                gScore.Add(pt, 100000);
                return 100000;
            }
        }

        Dictionary<SCR_Point, double> fScore = new()
        {
            { _robotStartPoint, H(_robotStartPoint) } // initial point has score 0
        };

        PriorityQueue<SCR_Point, double> nextPoint = new();
        nextPoint.Enqueue(_robotStartPoint, 1);

        while (open_set.Count > 0)
        {
            currentPoint = nextPoint.Dequeue();

            lookedAt[To1DArry(currentPoint)] = 1; //FIXME what do we even use lookedAt for ???

            if (currentPoint == _goalPoint)
            {
                //FIXME this shouldn't return we should like, do something else about it or something
                return ReconstructPath(path, current);
            }

            open_set.Remove(currentPoint);
            foreach (var (delta_x, delta_y, dist) in search_dirs)
            {
                SCR_Point neighbor = new(currentPoint.X + delta_x, currentPoint.Y + delta_y);

                if (neighbor.X < 0 || neighbor.X >= AStarConfig.ConfigSpaceWidth || neighbor.Y < 0 || neighbor.Y >= AStarConfig.ConfigSpaceHeight)
                {
                    continue;
                }

                var tentGScore = GetG(currentPoint) + D(neighbor, dist);
                if (tentGScore < GetG(neighbor))
                {
                    path[neighbor] = currentPoint;
                    gScore[neighbor] = tentGScore;
                    fScore[neighbor] = tentGScore + H(neighbor);

                    if (!open_set.Contains(neighbor))
                    {
                        open_set.Add(neighbor);
                        nextPoint.Enqueue(neighbor, fScore[neighbor]);
                    }
                }
            }
        }

        //TODO: publish debug image

        return Task.CompletedTask;
    }
}