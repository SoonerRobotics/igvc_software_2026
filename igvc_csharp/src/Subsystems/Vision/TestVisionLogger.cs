
using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Utils;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.src.Subsystems.Vision;

[Subsystem("TestVisionLogger", Disabled = true)]
public class TestVisionLoggerSubsystem() : SubsystemBase
{
    // subscriber frame channels
    private readonly Channel<ImageFrame> _leftChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    private readonly Channel<ImageFrame> _rightChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    private readonly Channel<ImageFrame> _combinedChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    private readonly Channel<ImageFrame> _combinedFilteredChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    // videowriters
    private VideoWriter _leftWriter;
    private VideoWriter _rightWriter;
    private VideoWriter _combinedWriter;
    private VideoWriter _combinedFilteredWriter;

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        InitWriters();

        // subscribers
        SubscribeImage(
            "left",
            OnLeftImageReceived,
            token
        );

        SubscribeImage(
            "right",
            OnRightImageReceived,
            token
        );

        SubscribeImage(
            "combined_view",
            OnCombinedImageReceived,
            token
        );

        SubscribeImage(
            "combined_filtered",
            OnFilteredImageReceived,
            token
        );

        _ = Task.Factory.StartNew(
            () => WriteCameraFrames(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        // _ = Task.Factory.StartNew(
        //     () => WriteAllFrames(token),
        //     token,
        //     TaskCreationOptions.LongRunning,
        //     TaskScheduler.Default
        // );

        // SetRobotMission(MissionEnum.Autonav);

        // SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    private void InitWriters()
    {
        //FIXME the FPS should be moved to configuration.cs or something maybe?
        //FIXME also the resolution definitely I think maybe? yes definitely, I spent more time than I'd like to admit trying to debug why writes were failing.
        _leftWriter = new VideoWriter(FileUtils.GetFileRelativeToRoot("resources/video/camera_left.avi"), VideoCaptureAPIs.ANY, FourCC.MJPG, 12, new Size(640, 480), true);
        _rightWriter = new VideoWriter(FileUtils.GetFileRelativeToRoot("resources/video/camera_right.avi"), VideoCaptureAPIs.ANY, FourCC.MJPG, 12, new Size(640, 480), true);
        _combinedWriter = new VideoWriter(FileUtils.GetFileRelativeToRoot("resources/video/camera_combined.avi"), VideoCaptureAPIs.ANY, FourCC.MJPG, 12, new Size(640+640, 480), true);
        _combinedFilteredWriter = new VideoWriter(FileUtils.GetFileRelativeToRoot("resources/video/filtered_combined.avi"), VideoCaptureAPIs.ANY, FourCC.MJPG, 12, new Size(640+640, 480), true);

        if (!_leftWriter.IsOpened() || !_rightWriter.IsOpened())
        {
            Logger.LogError("Failed to create video writers for vision test subsystem!");
        }
    }

    private Task OnLeftImageReceived(ImageFrame frame, CancellationToken token)
    {
        _leftChannel.Writer.TryWrite(frame);

        return Task.CompletedTask;
    }

    private Task OnRightImageReceived(ImageFrame frame, CancellationToken token)
    {
        _rightChannel.Writer.TryWrite(frame);

        SetOperatingState(SubsystemState.Operating);

        return Task.CompletedTask;
    }

    private Task OnCombinedImageReceived(ImageFrame frame, CancellationToken token)
    {
        _combinedChannel.Writer.TryWrite(frame);

        return Task.CompletedTask;
    }

    private Task OnFilteredImageReceived(ImageFrame frame, CancellationToken token)
    {
        _combinedFilteredChannel.Writer.TryWrite(frame);

        return Task.CompletedTask;
    }

    private async Task WriteCameraFrames(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                var leftFrame = await _leftChannel.Reader.ReadAsync(token);
                var rightFrame = await _rightChannel.Reader.ReadAsync(token);

                var leftImage = CvUtils.AsMat(leftFrame);
                var rightImage = CvUtils.AsMat(rightFrame);

                _leftWriter.Write(leftImage);
                _rightWriter.Write(rightImage);

                // Logger.LogInformation("leftFrame size: {}", leftImage.Size());

                if (leftImage.Empty() || rightImage.Empty())
                {
                    Logger.LogError("Camera frames are empty!");
                }

                // Logger.LogDebug("WROTE FRAMES!");
            }
        }
        catch (OperationCanceledException)
        {
            // Expected on shutdown
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Test video logging task crashed");
        }
    }

    private async Task WriteAllFrames(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                //TODO: feeler images, indiviaul filtered frames, A* debug, etc.
                var combinedFrame = await _combinedChannel.Reader.ReadAsync(token);
                var filteredFrame = await _combinedFilteredChannel.Reader.ReadAsync(token);

                var combinedImage = CvUtils.AsMat(combinedFrame);
                var filteredImage = CvUtils.AsMat(filteredFrame);

                _combinedWriter.Write(combinedImage);
                _combinedFilteredWriter.Write(filteredImage);

                // Logger.LogInformation("filtered size: {}", filteredImage.Size());
                // Logger.LogInformation("filtered channels: {}", filteredImage.Channels());

                if (combinedImage.Empty() || filteredImage.Empty())
                {
                    Logger.LogError("Camera frames are empty!");
                }
            }
        }
        catch (OperationCanceledException)
        {
            // Expected on shutdown
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Test video logging task crashed");
        }
    }

    public override Task Shutdown()
    {
        _leftWriter.Release();
        _rightWriter.Release();

        _leftWriter.Dispose();
        _rightWriter.Dispose();

        return Task.CompletedTask;
    }
}