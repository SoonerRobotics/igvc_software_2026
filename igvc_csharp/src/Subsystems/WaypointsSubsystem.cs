using Google.FlatBuffers;
using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Core.Units;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using WaypointConfig = igvc_csharp.Configuration.WaypointSubsystem;

namespace igvc_csharp.src.Subsystems;

public enum WaypointSetEnum
{
    None,
    Qualification,
    Selfdrive,
    Equad,
    Practice,
    Autonav,
    Simulation
}

public enum WaypointDirectionEnum
{
    North,
    South
}

[Subsystem("WaypointsSubsystem", Disabled = false)]
public class WaypointsSubsystem(CanbusSubsystem canbus) : SubsystemBase
{
    private RobotPosition? _position;
    private ulong _runStartTime = 0;
    private ulong _waypointTimeStart = 0;
    private LatLng? _startGpsPos;
    private Dictionary<WaypointSetEnum, List<LatLng>> _waypointsDict = [];

    private int _waypointIndex = 0;
    private bool _waypointsFinished = false;

    [Config("waypoints.set")]
    public WaypointSetEnum _waypointSet = WaypointSetEnum.None;

    [Config("waypoints.direction")]
    public WaypointDirectionEnum _waypointDirection = WaypointDirectionEnum.North;

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);
        ReadWaypointsFile();
        Subscribe<ArcClientConnectedEvent>((evt, ct) =>
        {
            BroadcastWaypointState();
            return Task.CompletedTask;
        }, token);
        SetOperatingState(SubsystemState.Ready);
        return Task.CompletedTask;
    }

    private Task ReadWaypointsFile()
    {
        _waypointsDict.Clear();
        int count = 0;

        try
        {
            using var file = new StreamReader(FileUtils.GetFileRelativeToRoot(WaypointConfig.WaypointsFilename));

            file.ReadLine(); // skip header

            string? line;
            while ((line = file.ReadLine()) != null)
            {
                var tokens = line.Split(",");
                if (tokens.Length < 3) continue;

                var set = Enum.Parse<WaypointSetEnum>(tokens[0], ignoreCase: true);
                var point = new LatLng(double.Parse(tokens[1]), double.Parse(tokens[2]));

                if (!_waypointsDict.ContainsKey(set))
                    _waypointsDict[set] = [];

                _waypointsDict[set].Add(point);
                count++;
            }
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Failed to read waypoints file! Waypoints will not work!");
            SetOperatingState(SubsystemState.Errored);
            return Task.CompletedTask;
        }

        _waypointIndex = 0;
        Logger.LogInformation("Read {Count} waypoints", count);

        if (count < 1)
        {
            Logger.LogWarning("No waypoints read, waypoint messages will not be published!");
        }

        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        if (updated.Mode == RobotModeEnum.Autonomous)
        {
            if (updated.MotionAllowed && _runStartTime == 0)
                TryStartRun();
        }
        else
        {
            Reset();
        }

        return Task.CompletedTask;
    }

    private void TryStartRun()
    {
        if (_position == null)
            return; // Not an error — just waiting for first GPS fix

        Logger.LogDebug("Starting waypoints run");
        _runStartTime = TimeUtils.Now();
        _startGpsPos = new LatLng(_position.Coordinates.Latitude, _position.Coordinates.Longitude);
        _waypointsFinished = false;
        SetOperatingState(SubsystemState.Operating);
    }

    private void Reset()
    {
        if (State != SubsystemState.Ready)
            SetOperatingState(SubsystemState.Ready);

        ReadWaypointsFile();
        _runStartTime = 0;
        _startGpsPos = null;
        _waypointSet = WaypointSetEnum.None;
        _waypointDirection = WaypointDirectionEnum.North;
        _waypointsFinished = false;
        _waypointTimeStart = 0;
    }

    public override void OnPositionChanged(RobotPosition position)
    {
        _position = position;

        // If we're in autonomous but couldn't start yet due to missing GPS, try again now
        if (_runStartTime == 0
            && BaseRobot.Instance?.State.Mode == RobotModeEnum.Autonomous
            && BaseRobot.Instance?.State.MotionAllowed == true)
        {
            TryStartRun();
        }

        if (_runStartTime == 0)
            return;

        if (!_waypointsFinished && _waypointSet == WaypointSetEnum.None)
            TryPickWaypointSet(position);

        if (!_waypointsFinished && _waypointSet != WaypointSetEnum.None && _waypointsDict.Count != 0)
            CheckWaypointReached(position);

        BroadcastWaypointState();
    }

    private void TryPickWaypointSet(RobotPosition position)
    {
        if ((TimeUtils.Now() - _runStartTime) < WaypointConfig.GpsWaitTime)
            return;

        // TODO: pick set based on mission/GPS region when competition logic is needed
        // _waypointSet = WaypointSetEnum.Qualification;
        _waypointSet = WaypointSetEnum.Autonav;
        _waypointDirection = WaypointDirectionEnum.North;
        _waypointIndex = 0;
        _waypointsFinished = false;
        _waypointTimeStart = 0;

        Logger.LogInformation(
            "Picked [{Set}] waypoints, direction [{Dir}], tracking {Count} points",
            _waypointSet, _waypointDirection, _waypointsDict[_waypointSet].Count);

        PublishCurrentWaypoint();
        PublishAudio("waypoints-start.mp3");
        canbus.SafetyLights.FlashWaypointFollow(CancellationToken.None);
        BroadcastWaypointState();
    }

    private void CheckWaypointReached(RobotPosition position)
    {
        var goalPoint = _waypointsDict[_waypointSet][_waypointIndex];
        var dist = position.Coordinates.Distance(goalPoint);
        if (dist.To(DistanceUnit.Meters) >= WaypointConfig.WaypointPopDist)
            return;

        if (_waypointTimeStart == 0)
        {
            _waypointTimeStart = TimeUtils.Now();
            return;
        }

        if ((TimeUtils.Now() - _waypointTimeStart) < WaypointConfig.WaypointPopTime)
            return;

        // Dwell time elapsed — advance to the next waypoint
        _waypointTimeStart = 0;
        _waypointIndex += _waypointDirection == WaypointDirectionEnum.North ? 1 : -1;

        canbus.SafetyLights.FlashWaypointReached(CancellationToken.None);
        if (_waypointIndex < 0 || _waypointIndex >= _waypointsDict[_waypointSet].Count)
        {
            _waypointsFinished = true;
            Logger.LogInformation("Reached end of waypoints list");
            return;
        }

        Logger.LogInformation("Waypoint reached — heading to waypoint {Index}", _waypointIndex);
        PublishCurrentWaypoint();
        PublishAudio("waypoint-hit.mp3");
        BroadcastWaypointState();
    }

    private void PublishCurrentWaypoint()
    {
        var point = _waypointsDict[_waypointSet][_waypointIndex];
        var builder = new FlatBufferBuilder(128);
        var setOffset = builder.CreateString(_waypointSet.ToString());
        var msg = Waypoint.CreateWaypoint(
            builder,
            TimeUtils.Now(),
            setOffset,
            _waypointIndex,
            (uint)_waypointsDict[_waypointSet].Count,
            point.Latitude,
            point.Longitude
        );
        builder.Finish(msg.Value);
        EventBus.Instance.Publish(new MessageWrapperEvent(
            MessageWrapper.From(MessageType.Waypoint, builder.SizedByteArray())));
    }

    private void BroadcastWaypointState()
    {
        if (_waypointSet == WaypointSetEnum.None || !_waypointsDict.ContainsKey(_waypointSet))
            return;

        var waypoints = _waypointsDict[_waypointSet];
        var current = _position != null && !_waypointsFinished
            ? waypoints[_waypointIndex]
            : (LatLng?)null;

        double? distanceMeters = null;
        double? bearingDegrees = null;

        if (_position != null && current != null)
        {
            distanceMeters = GeoUtils.LatLngDistance(_position.Coordinates, current).To(DistanceUnit.Meters); 
            bearingDegrees = GeoUtils.HeadingToPosition(_position.Coordinates, current).To(AngleUnit.Degrees);
        }

        var payload = new
        {
            waypointSet = _waypointSet.ToString(),
            direction = _waypointDirection.ToString(),
            currentIndex = _waypointIndex,
            finished = _waypointsFinished,
            distanceMeters,
            bearingDegrees,
            target = current != null ? new { lat = current.Latitude, lng = current.Longitude } : null,
            waypoints = waypoints.Select((wp, i) => new { lat = wp.Latitude, lng = wp.Longitude, index = i }).ToList()
        };

        EventBus.Instance.Publish(new MessageWrapperEvent(
            ArcUtils.CreateArcData_Json("waypoint_state", payload)));
    }

    private void PublishAudio(string filename)
    {
        var builder = new FlatBufferBuilder(128);
        var fileOffset = builder.CreateString(filename);
        var msg = AudibleFeedback.CreateAudibleFeedback(builder, fileOffset, false);
        builder.Finish(msg.Value);
        EventBus.Instance.Publish(new MessageWrapperEvent(
            MessageWrapper.From(MessageType.AudibleFeedback, builder.SizedByteArray())));
    }
}