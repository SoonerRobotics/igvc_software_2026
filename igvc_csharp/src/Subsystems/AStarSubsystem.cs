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
    private SCR_Point _goalPoint;
    private List<SCR_Point> _path = [];
    private SCR_Point _robotStartPoint = new(48, 78); //FIXME rewrite in terms of config space dimensions / make configurable
    private LatLng _waypoint;

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        //TODO: subscribers / publishers

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    public void Reset()
    {
        //TODO

        //TODO reset _robotStartPoint?

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

    public void FindGoalPoint()
    {
        // self.performance.start("Smellification")

        // grid_data = msg.data
        var workingGoalPoint = new SCR_Point(40, 78); //TODO rewrite in terms of config space dimensions
        double workingCost = -1000;

        HashSet<SCR_Point> frontier = [_robotStartPoint]; //FIXME we should try and re-use this for the A* or something.
        HashSet<SCR_Point> explored = [];

        //FIXME
        // if self.config.getBool(CONFIG_USE_ONLY_WAYPOINTS) == True:
        //     grid_data = [0] * len(msg.data)


        //FIXME do a better null check or something
        if (_waypoint.Latitude != 0)
        {
            //FIXME
            // north_to_gps = (next_waypoint[0] - self.position.latitude) * self.latitudeLength
            // west_to_gps = (self.position.longitude - next_waypoint[1]) * self.longitudeLength
            // heading_to_gps = math.atan2(west_to_gps, north_to_gps) % (2 * math.pi)

            // pathingDebug = PathingDebug()
            // pathingDebug.desired_heading = heading_to_gps
            // pathingDebug.desired_latitude = next_waypoint[0]
            // pathingDebug.desired_longitude = next_waypoint[1]
            // pathingDebug.distance_to_destination = north_to_gps ** 2 + west_to_gps ** 2
            // wp1d = []
            // for wp in self.waypoints:
            //     wp1d.append(wp[0])
            //     wp1d.append(wp[1])
            // pathingDebug.waypoints = wp1d
            // self.debugPublisher.publish(pathingDebug)
        }

        int depth = 0;
        //FIXME add cancellation token check?
        while (depth < 50 && frontier.Count > 0)
        {
            var frontierCopy = frontier; //TODO make this a deep copy
            foreach (var point in frontierCopy)
            {
                //FIXME rewrite in terms of config space dimensions
                // also FIXME make the weights configurable
                double cost = (80 - point.Y) * 1.3 + depth * 2.2;

                // if len(self.waypoints) > 0:
                //     heading_err_to_gps = abs(self.getAngleDifference(self.position.theta + math.atan2(40 - x, 80 - y), heading_to_gps)) * 180 / math.pi
                //     cost -= max(heading_err_to_gps, 10)

                if (cost > workingCost)
                {
                    workingCost = cost;
                    workingGoalPoint = point;
                }

                frontier.Remove(point);
                explored.Add(point);

                //REALLY BIG FIXME: these !explored.Contains(point) need to be like, !explored.Contains(point.X + 1) typa thing y'know what I'm sayin?

                if (point.Y > 0 && configSpace[point] < 50 && !explored.Contains(point))
                {
                    // we're in image coordinates, so negative Y means upwards in the image, means forwards for the robot
                    frontier.Add(new SCR_Point(point.X, point.Y - 1));
                }

                //FIXME rewrite in terms of config space dimensions
                if (point.X < 79 && configSpace[point] < 50 && !explored.Contains(point))
                {
                    frontier.Add(new SCR_Point(point.X + 1, point.Y));
                }

                if (point.X > 0 && configSpace[point] < 50 && !explored.Contains(point))
                {
                    frontier.Add(new SCR_Point(point.X - 1, point.Y));
                }
            }

            depth += 1;
        }

        // self.costMap = grid_data
        // self.bestPosition = temp_best_pos
        // self.performance.end("Smellification")

        _goalPoint = workingGoalPoint;
    }

    public void FindPath()
    {
        FindGoalPoint();

        // looked_at = np.zeros((80, 80))
        HashSet<SCR_Point> open_set = [_robotStartPoint];
        List<SCR_Point> path = [];
        int[] search_dirs = []; //TODO FIXME
        SCR_Point current;

        // self.performance.start("A*")

        // for x in range(-1, 2):
        //     for y in range(-1, 2):
        //         if x == 0 and y == 0:
        //             continue
        //         search_dirs.append((x,y,math.sqrt(x ** 2 + y ** 2)))

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

        SCR_Point next_current = [(1, start)]; //FIXME???
        while (open_set.Count > 0)
        {
            current = heappop(next_current)[1];

            looked_at[current.X, current.Y] = 1;

            if (current == _goalPoint)
            {
                return ReconstructPath(path, current);
            }

            open_set.Remove(current);
            foreach ((delta_x, delta_y, dist) in search_dirs) {
                SCR_Point neighbor = (current.X + delta_x, current.Y + delta_y);

                if (neighbor.X < 0 || neighbor.X >= width || neighbor.Y < 0 || neighbor.Y >= height) {
                    continue;
                }

                var tentGScore = GetG(current) + D(neighbor, dist);
                if (tentGScore < GetG(neighbor))
                {
                    path[neighbor] = current;
                    gScore[neighbor] = tentGScore;
                    fScore[neighbor] = tentGScore + H(neighbor);
                    
                    if (!open_set.Contains(neighbor))
                    {
                        open_set.Add(neighbor);
                        heappush(next_current, (fScore[neighbor], neighbor));
                    }
                }
            }
        }
    }
}