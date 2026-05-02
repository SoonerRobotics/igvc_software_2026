
using igvc_csharp.Core;
using igvc_csharp.Core.Hardware;
using igvc_csharp.Core.Units;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Microsoft.Extensions.Logging;
using WaypointConfig = igvc_csharp.Configuration.WaypointSubsystem;

namespace igvc_csharp.src.Subsystems.Control;

[Subsystem("WaypointsSubsystem", Disabled = false)]
public class WaypointsSubsystem() : SubsystemBase
{
    // GPS stuff
    private VectorNavReport _position;
    private ulong _runStartTime = 0;
    private ulong _waypointTimeStart = 0;
    private LatLng? _startGpsPos;
    private Dictionary<string, List<LatLng>> _waypointsDict = [];
    private int _waypointIndex = 0;
    private string _direction = ""; //FIXME make this an enum or something

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        ReadWaypointsFile();

        SubscribeMessage<VectorNavReport>(
            MessageType.Gps,
            OnPositionReceived,
            token
        );

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    private Task ReadWaypointsFile()
    {
        int numWaypoints = 0;
        // === read waypoints from file === (copied and pasted from 2025's C++ feeler code, which was copied from 2024's feat/astar_rewrite_v3 branch)
        string line;

        using (StreamReader waypointsFile = new(FileUtils.GetFileRelativeToRoot(WaypointConfig.WaypointsFilename)))
        {
            // skip the first line
            line = waypointsFile.ReadLine();
            while ((line = waypointsFile.ReadLine()) != null)
            {
                var tokens = line.Split(",");

                LatLng point = new(
                    double.Parse(tokens[1]), //FIXME we should add a .strip() or something here
                    double.Parse(tokens[2])
                );

                // waypoints are stored like {"north":[GPSPoint, GPSPoint]}
                _waypointsDict[tokens[0]].Add(point);
                numWaypoints++;
            }
        }
        _waypointIndex = 0;
        // === /read waypoints ===

        //TODO add like, per-label waypoint counts? theoreticlly we don't have to modify them that much but also idk
        Logger.LogInformation("Number of waypoints read: " + numWaypoints);

        if (numWaypoints < 1)
        {
            Logger.LogWarning("No waypoints read! Waypoint messages will not be published!");
        }

        return Task.CompletedTask;
    }

    public override Task OnRobotModeChanged(RobotModeEnum old, RobotModeEnum current)
    {
        //FIXME do we want this to be able to run in manual too? for testing purposes?
        // also FIXME what about how this works in the simulator how is it going to work in the simulator?
        if (current == RobotModeEnum.Autonomous)
        {
            // this will get called when mobility gets updated too, so we can check it here
            if (Robot.Instance.State.MotionAllowed && _runStartTime == 0)
            {
                _runStartTime = TimeUtils.Now();
                _startGpsPos = new LatLng(_position.Latitude, _position.Longitude);
                SetOperatingState(SubsystemState.Operating);
            }
        }
        else
        {
            if (State != SubsystemState.Ready)
            {
                SetOperatingState(SubsystemState.Ready);
            }

            //FIXME do we need to reset it here? I feel like we should... I can see accidentally forgetting to and then doing a run
            // but the robot thinks the starting position is over on the practice course
            _runStartTime = 0;
        }

        return Task.CompletedTask;
    }

    private Task CheckDirection()
    {
        if ((TimeUtils.Now() - _runStartTime) > WaypointConfig.GpsWaitTime)
        {

            //FIXME add a set of practice waypoints too, and even like OU e-quad waypoints we can can switch too based on the lat/lon of _startPos
            // ALSO SELF-DRIVE COURSE WAYPOINTS (idk if those are allowed though but we can have a .IsRobotInSelfDriveMode() check)

            // then pick a set of waypoints based on which direction we are heading
            double heading_degrees = LatLng.TravelHeading(_startGpsPos, new LatLng(_position.Latitude, _position.Longitude)).Value.To(AngleUnit.Degrees);
            if (120 < heading_degrees && heading_degrees < 240)
            {
                _direction = "compSouth";
                Logger.LogInformation("Picking south waypoints!");
            }
            else
            {
                _direction = "compNorth";
                Logger.LogInformation("Picking north waypoints!");
            }

            //TODO: we should flash safety lights to let operator know that the GPS waypoints are working / have been selected
        }

        return Task.CompletedTask;
    }

    private Task OnPositionReceived(VectorNavReport msg, CancellationToken token)
    {
        _position = msg;

        // don't do anything until we're running
        if (_runStartTime == 0)
        {
            return Task.CompletedTask;
        }

        if (_direction == "")
        {
            CheckDirection();
        }

        // waypoint reach detection
        else if (_waypointsDict.Count() != 0)
        {
            //TODO if we haven't published the first waypoint, we should publish it here after waypoint wait time or whatever

            var current_gps = new LatLng(msg.Latitude, msg.Longitude);
            var goalPoint = _waypointsDict[_direction][_waypointIndex];

            var dist = current_gps.Distance(goalPoint);

            // if we are close enough to the waypoint, and we aren't going to cause an out-of-bounds index error
            if (dist.To(DistanceUnit.Meters) < WaypointConfig.WaypointPopDist && _waypointIndex < (_waypointsDict[_direction].Count - 2))
            {
                if (_waypointTimeStart == 0)
                {
                    // start the clock for how long we have to be close to it
                    _waypointTimeStart = TimeUtils.Now();
                }
                else if ((TimeUtils.Now() - _waypointTimeStart) > WaypointConfig.WaypointPopTime)
                {
                    //FIXME we should add like, waypoint pass detection to just go to the next one if we've gone past it maybe? so we don't get stuck in a loop of death or something

                    // then go to the next waypoint
                    _waypointIndex++;

                    Logger.LogInformation("Waypoint Reached! Heading to next...");
                    _waypointTimeStart = 0;

                    //TODO: flash safety lights green here for visual debug information n stuff

                    //TODO: publish a Waypoint message for the next waypoint
                }
            }
        }

        return Task.CompletedTask;
    }
}