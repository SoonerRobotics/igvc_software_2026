
using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Core.Units;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using MapConfig = igvc_csharp.Configuration.BigMapSubsystem;
using AStarConfig = igvc_csharp.Configuration.AStarSubsystem;
using OpenCvSharp;
using igvc_csharp.Events;


namespace igvc_csharp.src.Subsystems;

[Subsystem("BigMapSubsystem", Disabled = false)]
public class BigMapSubsystem() : SubsystemBase
{
    // map stuff
    //FIXME should this be like, a byte instead?
    private uint[,] _bigMap = new uint[
        (int)Math.Round(MapConfig.AutonavCourseWidth.To(DistanceUnit.Feet)),
        (int)Math.Round(MapConfig.AutonavCourseHeight.To(DistanceUnit.Feet))
    ]; // each pixel is a foot

    // GPS stuff
    private VectornavReport _position;
    private VectornavReport _lastPosition;
    private LatLng? _goalPoint;
    private LatLng? _startPos;

    // OpenCV stuff
    private readonly Channel<ImageFrame> _maskFrameChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    private OpenCvImageWindow _window = new("Big Map");

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

        Logger.LogInformation("On Config Space Received!!!");

        LatLng lastPos = new(_lastPosition.Latitude, _lastPosition.Longitude);

        if (_startPos == null)
        {
            return Task.CompletedTask;
        }
        var (east_m, north_m) = GeoUtils.GenerateOffsetMeters(_startPos, lastPos);

        // robot's local position within the big grid map array
        int x = (int)new Distance(east_m).To(DistanceUnit.Feet);
        int y = (int)new Distance(north_m).To(DistanceUnit.Feet);

        //FIXME what if x and y are negative?
        Logger.LogInformation("Data length: {}", msg.DataLength);


        var OneDArr = msg.GetDataArray();
        Logger.LogInformation("Received ConfigSpace with size: {}", OneDArr.Length);
        // for (int i = 0; i < OneDArr.Length; i++)
        // {
        //     //TODO: figure out how to handle robot rotation
        //     int localX = i % AStarConfig.ConfigSpaceWidth;
        //     int localY = i % AStarConfig.ConfigSpaceHeight; //FIXME this is wrong for sure I think

        //     // adjust based on robot position
        //     localX += MapConfig.RobotStartPosition.Item1 + x; //FIXME do these need to be plus or minus?
        //     localY += MapConfig.RobotStartPosition.Item2 + y;

        //     //TODO: try catch for index out of bounds or something

        //     _bigMap[localX, localY] += OneDArr[i]; //TODO do some like, scaling shenaniganry, or only increase it by 1 every time it's over Config.ObstacleThreshold
        // }

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
                    //TODO publish some kinda map message ???

                    if (_bigMap.Length == 0)
                    {
                        await Task.Delay(1000, token);
                        continue;
                    }

                    // publish debug image
                    var mat = Mat.FromArray(_bigMap);

                    var frameBytes = CvUtils.FromMat(mat);
                    var newFrame = MessageConstructor.CreateImageFrame(
                        (uint)mat.Width,
                        (uint)mat.Height,
                        "debug_feelers",
                        frameBytes
                    );

                    var wrappedFrame = MessageWrapper.From(
                        MessageType.ImageFrame,
                        newFrame.ByteBuffer.ToFullArray()
                    );

                    EventBus.Instance.Publish(new MessageWrapperEvent(wrappedFrame));

                    _window.EnqueueJpeg(mat.ToBytes());

                    mat.Dispose();
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
            Logger.LogError(ex, "Big Map subsystem crashed");
            _window.Dispose();
            Cv2.DestroyAllWindows();
        }
    }

    public override Task Shutdown()
    {
        _window.Dispose();
        Cv2.DestroyAllWindows();

        return Task.CompletedTask;
    }
}