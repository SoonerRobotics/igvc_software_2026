using System.Diagnostics;
using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Subsystems.Simulator;

[Subsystem("FakeCameraSubsystem", Disabled = false)]
public class FakeCameraSubsystem : SubsystemBase
{
    private readonly Channel<ImageFrame> _frameChannel = Channel.CreateBounded<ImageFrame>(new BoundedChannelOptions(1)
    {
        SingleReader = true,
        SingleWriter = true,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        _ = Task.Factory.StartNew(
            () => ImagePublishingTask(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    private async Task ImagePublishingTask(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                SetOperatingState(SubsystemState.Operating);

                var frame = await _frameChannel.Reader.ReadAsync(token);

                var mat = CvUtils.AsMat(frame);

                var frameBytes = CvUtils.FromMat(mat);
                var newFrame = MessageConstructor.CreateImageFrame(
                    frame.Width,
                    frame.Height,
                    "front_view",
                    frameBytes
                );

                var wrappedFrame = MessageWrapper.From(
                    MessageType.ImageFrame,
                    newFrame.ByteBuffer.ToFullArray()
                );

                EventBus.Instance.Publish(new MessageWrapperEvent(wrappedFrame));

                mat.Dispose();

                // await time.sleep(FPS);
            }
        }
        catch (OperationCanceledException)
        {
            // Expected on shutdown
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Image publishing task crashed");
            SetOperatingState(SubsystemState.Errored);
        }
    }
}