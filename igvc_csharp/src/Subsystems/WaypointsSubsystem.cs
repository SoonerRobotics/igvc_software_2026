
using Google.FlatBuffers;
using igvc_csharp.Core;
using igvc_csharp.Core.Hardware;
using igvc_csharp.Core.Units;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using WaypointConfig = igvc_csharp.Configuration.WaypointSubsystem;

namespace igvc_csharp.src.Subsystems;

[Subsystem("WaypointsSubsystem", Disabled = false)]
public class WaypointsSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    // GPS stuff
    private VectorNavReport? _position;
    private ulong _runStartTime = 0;
    private ulong _waypointTimeStart = 0;
    private LatLng? _startGpsPos;
    private Dictionary<string, List<LatLng>> _waypointsDict = [];
    private int _waypointIndex = 0;
    private string _waypointSet = ""; //FIXME make this an enum or something?
    private int _waypointDirection = 0;
    private bool _waypointsFinished = false;

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        ReadWaypointsFile(token);

        SubscribeMessage<VectorNavReport>(
            MessageType.VectorNav,
            OnPositionReceived,
            token
        );

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    private Task ReadWaypointsFile(CancellationToken token)
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
                    double.Parse(tokens[1]),
                    double.Parse(tokens[2])
                );

                if (!_waypointsDict.ContainsKey(tokens[0]))
                {
                    _waypointsDict[tokens[0]] = [];
                }

                // waypoints are stored like {"north":[GPSPoint, GPSPoint]}
                _waypointsDict[tokens[0]].Add(point);
                numWaypoints++;
            }
        }
        _waypointIndex = 0;
        // === /read waypoints ===

        Logger.LogInformation("Number of waypoints read: " + numWaypoints);

        if (numWaypoints < 1)
        {
            Logger.LogWarning("No waypoints read! Waypoint messages will not be published!");
        }

        return Task.CompletedTask;
    }

    public override Task OnRobotModeChanged(RobotModeEnum old, RobotModeEnum current)
    {
        if (current == RobotModeEnum.Autonomous)
        {
            // this will get called when mobility gets updated too, so we can check it here
            if (BaseRobot.Instance.State.MotionAllowed && _runStartTime == 0)
            {
                if (_position.HasValue)
                {
                    _runStartTime = TimeUtils.Now();
                    _startGpsPos = new LatLng(_position.Value.Latitude, _position.Value.Longitude);
                    _waypointsFinished = false;
                    SetOperatingState(SubsystemState.Operating);
                }
                else
                {
                    Logger.LogError("GPS Subsystem is not running! WaypointSubsystem cannot start operating!");
                }
            }
        }
        else
        {
            if (State != SubsystemState.Ready)
            {
                SetOperatingState(SubsystemState.Ready);
            }

            _runStartTime = 0;
            _startGpsPos = null;
            _waypointSet = "";
            _waypointDirection = 0;
            _waypointsFinished = false;
            _waypointTimeStart = 0;
        }

        return Task.CompletedTask;
    }

    private Task CheckDirection(VectorNavReport msg, CancellationToken token)
    {
        if ((TimeUtils.Now() - _runStartTime) > WaypointConfig.GpsWaitTime)
        {
            // pick the set of waypoints based on robot state and GPS position information
            if (BaseRobot.Instance.State.Mission == MissionEnum.Selfdrive)
            {
                _waypointSet = "selfdrive";
            }
            else if (msg.Latitude < Configuration.WaypointSubsystem.EquadLatitude)
            {
                _waypointSet = "equad";
            }
            else if (msg.Longitude > Configuration.WaypointSubsystem.PracticeLongitude)
            {
                _waypointSet = "practice";
            }
            else
            {
                _waypointSet = "autonav";
            }


            // then pick a set of waypoints based on which direction we are heading
            double heading_degrees = LatLng.TravelHeading(_startGpsPos, new LatLng(msg.Latitude, msg.Longitude)).Value.To(AngleUnit.Degrees);
            if (120 < heading_degrees && heading_degrees < 240)
            {
                _waypointDirection = -1; // south
                // if we are going south, make the index the last element in the list and we will work backwards
                _waypointIndex = _waypointsDict[_waypointSet].Count() - 1;
            }
            else
            {
                _waypointDirection = 1; // north
                _waypointIndex = 0;
            }


            Logger.LogInformation("Picked [{}] waypoint set with direction [{}]! Tracking [{}] waypoints...", _waypointSet, _waypointDirection.ToString(), _waypointsDict[_waypointSet].Count);

            _waypointsFinished = false;
            _waypointTimeStart = 0;

            //FIXME should we publish the first waypoint message here then? instead of waiting for OnPositionReceived?

            canbus.SafetyLights.FlashTemporary(ColorUtils.Color.Blue, token, 2000);
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

        if (_waypointSet == "")
        {
            CheckDirection(msg, token);
        }

        // waypoint reach detection
        if (_waypointsDict.Count() != 0 && !_waypointsFinished)
        {
            var current_gps = new LatLng(msg.Latitude, msg.Longitude);
            var goalPoint = _waypointsDict[_waypointSet][_waypointIndex];

            var dist = current_gps.Distance(goalPoint);

            // if we are close enough to the waypoint
            if (dist.To(DistanceUnit.Meters) < WaypointConfig.WaypointPopDist)
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
                    _waypointIndex += _waypointDirection;

                    if (_waypointIndex < 0 || _waypointIndex + 1 > _waypointsDict[_waypointSet].Count())
                    {
                        // we've reached the end of the list, publish no more
                        _waypointsFinished = true;
                        Logger.LogInformation("Reached end of waypoints list!");
                    }
                    else
                    {
                        Logger.LogInformation("Waypoint Reached! Heading to next...");
                    }

                    _waypointTimeStart = 0;

                    canbus.SafetyLights.FlashTemporary(ColorUtils.Color.Green, token, 2000, 500);

                    // publish a Waypoint message for the next waypoint
                    var builder = new FlatBufferBuilder(128);
                    var msgOffset = Waypoint.CreateWaypoint(
                        builder,
                        TimeUtils.Now(),
                        new StringOffset(_waypointSet.Length), //FIXME does this actually put the string in there or is like... this not gonna work.
                        _waypointIndex,
                        (uint)_waypointsDict[_waypointSet].Count,
                        goalPoint.Latitude,
                        goalPoint.Longitude
                    );
                    builder.Finish(msgOffset.Value);
                    var waypointMsg = MessageWrapper.From(MessageType.Waypoint, builder.SizedByteArray());
                    EventBus.Instance.Publish(
                        new MessageWrapperEvent(waypointMsg)
                    );
                }
            }
        }

        return Task.CompletedTask;
    }
}