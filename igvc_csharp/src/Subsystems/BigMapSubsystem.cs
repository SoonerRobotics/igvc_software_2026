
using System.Reflection.PortableExecutable;
using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Core.Units;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using MapConfig = igvc_csharp.Configuration.BigMapSubsystem;


namespace igvc_csharp.src.Subsystems;

[Subsystem("BigMapSubsystem", Disabled = false)]
public class BigMapSubsystem() : SubsystemBase
{
    // map stuff
    //FIXME should this be like, a byte instead?
    private int[,] _bigMap = new int[MapConfig.AutonavCourseWidth, MapConfig.AutonavCourseHeight]; // each pixel is a foot

    // GPS stuff
    private VectornavReport _position;
    private VectornavReport _lastPosition;
    private LatLng? _goalPoint;
    private LatLng? _startPos;
    private ConfigSpace _configSpace; // ???

    // OpenCV stuff
    private readonly Channel<ImageFrame> _maskFrameChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        Reset();

        // subscribers
        SubscribeMessage<ConfigSpace>(
            MessageType.ConfigSpace,
            OnConfigSpaceReceived,
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

        _ = Task.Factory.StartNew(
            () => PublishMap(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    private void Reset()
    {
        for (int y = 0; y < _bigMap.GetLength(0); y++)
        {
            for (int x = 0; x < _bigMap.GetLength(1); x++)
            {
                _bigMap[x, y] = 0;
            }
        }

        SetOperatingState(SubsystemState.Ready);
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        //TODO: do something like reset the map or something

        if (!old.MotionAllowed && updated.MotionAllowed)
        {
            Reset();

            _startPos = new(_position.Latitude, _position.Longitude);
        }

        return Task.CompletedTask;
    }

    private Task OnConfigSpaceReceived(ConfigSpace msg, CancellationToken token)
    {
        SetOperatingState(SubsystemState.Operating);
        
        LatLng lastPos = new(_lastPosition.Latitude, _lastPosition.Longitude);
        var (east_m, north_m) = GeoUtils.GenerateOffsetMeters(_startPos, lastPos);

        var x = new Distance(east_m).To(DistanceUnit.Feet);
        var y = new Distance(north_m).To(DistanceUnit.Feet);

        //FIXME what if x and y are negative?

        for (int i = 0; i < _msg.data.length; i++)
        {
            //TODO sub array increase number or something or whatever
        }

        return Task.CompletedTask;
    }

    private Task OnPositionReceived(VectornavReport msg, CancellationToken token)
    {
        _lastPosition = _position;
        _position = msg;

        return Task.CompletedTask;
    }

    private Task OnWaypointReceived(Waypoint msg, CancellationToken token)
    {
        _goalPoint = new(msg.Latitude, msg.Longitude);

        return Task.CompletedTask;
    }

    private async Task PublishMap(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                if (State == SubsystemState.Operating)
                {
                    //TODO publish message ???
                }
                else
                {
                    //FIXME I think there's a better way to do this?
                    await Task.Delay(1000, token);
                }
            }
        }
        catch (OperationCanceledException)
        {
            // Expected on shutdown
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Feeler task crashed");
        }
    }
}