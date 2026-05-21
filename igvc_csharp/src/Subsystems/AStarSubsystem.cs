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


namespace igvc_csharp.src.Subsystems;

[Subsystem("AStarSubsystem", Disabled = true)]
public class AStarSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    private SCR_Point _goalPoint;
    private SCR_Point _robotStartPoint = new(AStarConfig.ConfigSpaceWidth / 2, AStarConfig.ConfigSpaceHeight - 2);
    private LatLng? _waypoint;
    private LatLng? _position;
    private LatLng? _lastPosition;
    private double _heading = 0;
    private byte[] _configSpace = [];
    private PurePursuit _purePursuit = new();

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        // subscribers
        SubscribeMessage<VectornavReport>(
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


        _ = Task.Factory.StartNew(
            () => SendMotorCommands(token),
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
        _robotStartPoint = new(AStarConfig.ConfigSpaceWidth / 2, AStarConfig.ConfigSpaceHeight - 2);
        _waypoint = null;
        _position = null;
        _lastPosition = null;
        _heading = 0;
        _configSpace = [];

        SetOperatingState(SubsystemState.Ready);
    }

    // gets the index of a 2D point in the 1D config space array
    private static int To1DArry(SCR_Point pt)
    {
        return (pt.Y * AStarConfig.ConfigSpaceWidth) + pt.X;
    }

    // expects both in radians
    private static double GetAngleDifference(double a, double b)
    {
        return a - b; //FIXME
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        // on an actual mode change FIXME we might want to reset for mob start/stop too?
        if (old != updated)
        {
            if (updated.Mode == RobotModeEnum.Autonomous)
            {
                Reset();
            }
        }

        //TODO: are we supposed to control the safety lights as well? or only for debug information?

        return Task.CompletedTask;
    }

    private Task OnPositionReceived(VectornavReport msg, CancellationToken token)
    {
        _position = new LatLng(msg.Latitude, msg.Longitude);

        if (_lastPosition != null)
        {
            _heading = GeoUtils.EstimateHeading(_lastPosition, _position).Value.To(AngleUnit.Radians);
        }

        _lastPosition = _position;

        return Task.CompletedTask;
    }

    private Task OnWaypointReceived(Waypoint msg, CancellationToken token)
    {
        _goalPoint = new(msg.Latitude, msg.Longitude);

        return Task.CompletedTask;
    }

    private Task OnConfigSpaceReceived(ConfigSpace msg, CancellationToken token)
    {
        if (AStarConfig.UseAStar)
        {
            //TODO FIXME this may need to be a deep copy or something?
            _configSpace = msg.GetDataArray();

            FindPath(token);
        }

        return Task.CompletedTask;
    }

    private Task FindGoalPoint(CancellationToken token)
    {
        // self.performance.start("Smellification")

        var workingGoalPoint = new SCR_Point(_robotStartPoint.X, _robotStartPoint.Y);
        double workingCost = -1000;

        HashSet<SCR_Point> frontier = [_robotStartPoint]; //FIXME we should try and re-use this for the A* or something.
        HashSet<SCR_Point> explored = [];

        // if we are going to ignore obstacles, then zero out the config space
        if (AStarConfig.UseOnlyWaypoints)
        {
            for (int i = 0; i < _configSpace.Length; i++)
            {
                _configSpace[i] = 0;
            }
        }

        int depth = 0;
        while (depth < AStarConfig.SmellyMaxDepth && frontier.Count > 0 && !token.IsCancellationRequested)
        {
            var frontierCopy = frontier; //TODO make this a deep copy
            foreach (var point in frontierCopy)
            {
                double cost = (AStarConfig.ConfigSpaceHeight - point.Y) * AStarConfig.SmellyDistanceWeight + (depth * AStarConfig.SmellyDepthWeight);

                //FIXME better null check or something
                if (_goalPoint != null)
                {
                    var heading_err_to_gps = GetAngleDifference(_heading, GeoUtils.EstimateHeading(_position, _waypoint).Value.To(AngleUnit.Radians));
                    cost -= Math.Max(heading_err_to_gps, 10);
                }

                if (cost > workingCost)
                {
                    workingCost = cost;
                    workingGoalPoint = point;
                }

                frontier.Remove(point);
                explored.Add(point);

                if (point.Y > 0 && _configSpace[To1DArry(point)] < AStarConfig.ObstacleThreshold && !explored.Contains(new SCR_Point(point.X, point.Y - 1)))
                {
                    // we're in image coordinates, so negative Y means upwards in the image, means forwards for the robot
                    frontier.Add(new SCR_Point(point.X, point.Y - 1));
                }

                if (point.X < (AStarConfig.ConfigSpaceWidth - 1) && _configSpace[To1DArry(point)] < AStarConfig.ObstacleThreshold && !explored.Contains(new SCR_Point(point.X + 1, point.Y)))
                {
                    frontier.Add(new SCR_Point(point.X + 1, point.Y));
                }

                if (point.X > 0 && _configSpace[To1DArry(point)] < AStarConfig.ObstacleThreshold && !explored.Contains(new SCR_Point(point.X - 1, point.Y)))
                {
                    frontier.Add(new SCR_Point(point.X - 1, point.Y));
                }
            }

            depth += 1;
        }

        // self.performance.end("Smellification")

        _goalPoint = workingGoalPoint;

        return Task.CompletedTask;
    }

    private Task ReconstructPath(Dictionary<SCR_Point, SCR_Point> p, CancellationToken token)
    {
        _purePursuit.Reset();

        bool reconstructed = false;
        var parent = p[_goalPoint];
        _purePursuit.AddPoint(_goalPoint);

        while (!reconstructed && !token.IsCancellationRequested)
        {
            parent = p[parent];

            if (parent == _robotStartPoint)
            {
                reconstructed = true; // ??? FIXME is this even correct?
            }

            _purePursuit.AddPoint(parent);
        }

        return Task.CompletedTask;
    }

    private Task FindPath(CancellationToken token)
    {
        if (BaseRobot.Instance.State.Mode != RobotModeEnum.Autonomous || BaseRobot.Instance.State.Mission != MissionEnum.Autonav)
        {
            return Task.CompletedTask;
        }

        SetOperatingState(SubsystemState.Operating);

        FindGoalPoint(token);

        if (token.IsCancellationRequested)
        {
            return Task.CompletedTask;
        }

        int[] lookedAt = new int[AStarConfig.ConfigSpaceWidth * AStarConfig.ConfigSpaceHeight];
        HashSet<SCR_Point> open_set = [_robotStartPoint];
        Dictionary<SCR_Point, SCR_Point> path = [];
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

        int D(SCR_Point pt, double d)
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

        while (open_set.Count > 0 && !token.IsCancellationRequested)
        {
            currentPoint = nextPoint.Dequeue();

            //TODO if the code doesn't break then remove this like because we never check it ???
            // lookedAt[To1DArry(currentPoint)] = 1; //FIXME what do we even use lookedAt for ???

            if (currentPoint == _goalPoint)
            {
                ReconstructPath(path, token);

                return Task.CompletedTask;
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

    private Task SendMotorCommands(CancellationToken token)
    {
        if (AStarConfig.UseAStar)
        {
            //TODO are we gonna run this in selfdrive too though?
            if (BaseRobot.Instance.State.Mode == RobotModeEnum.Autonomous && BaseRobot.Instance.State.Mission == MissionEnum.Autonav)
            {
                //TODO: convert _position to like, an actual local in-config-space-map position
                // SCR_Point localPos = new(_position.Latitude, _position.Longitude);
                SCR_Point localPos = _robotStartPoint;
                var point = _purePursuit.GetLookaheadPoint(localPos, AStarConfig.LookaheadRadius);

                double angleToPoint = 0; //TODO need to actually calculate the angle to turn at

                if (BaseRobot.Instance.State.MotionAllowed)
                {
                    //TODO: add a back-up sequence like in previous years
                    //TODO: lower our forward speed based on how much we have to turn but not as much as we had to for the Danger Zone
                    //TODO: clamp our max speed to some configurable value maybe?
                    //FIXME this is incorrect
                    canbus.MotorControl.SetVelocities(1.0, 0.0, angleToPoint);
                }
            }
        }

        return Task.CompletedTask;
    }
}