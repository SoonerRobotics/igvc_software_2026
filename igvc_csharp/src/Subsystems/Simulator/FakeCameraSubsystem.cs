using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Subsystems.Simulator;

[Subsystem("FakeCameraSubsystem", Disabled = false)]
public class FakeCameraSubsystem : SubsystemBase
{
    private VideoCapture? _video;

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        _video = new VideoCapture(FileUtils.GetFileRelativeToRoot(Configuration.FakeCameraSubsystemConfig.Filename));

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
        bool EOF = false;
        try
        {
            while (!token.IsCancellationRequested && !EOF)
            {
                SetOperatingState(SubsystemState.Operating);

                Mat combined = new();
                if (_video == null || !_video.Read(combined))
                {
                    EOF = true;
                    continue;
                }

                Mat leftMat = combined.SubMat(0, combined.Height, 0, combined.Width / 2);
                Mat rightMat = combined.SubMat(0, combined.Height, combined.Width / 2, combined.Width);

                var leftFrame = MessageConstructor.CreateImageFrame(
                    (uint)leftMat.Width,
                    (uint)leftMat.Height,
                    "left_view",
                    CvUtils.FromMat(leftMat)
                );

                var wrappedLeft = MessageWrapper.From(
                    MessageType.ImageFrame,
                    leftFrame.ByteBuffer.ToFullArray()
                );

                var rightFrame = MessageConstructor.CreateImageFrame(
                    (uint)rightMat.Width,
                    (uint)rightMat.Height,
                    "right_view",
                    CvUtils.FromMat(rightMat)
                );

                var wrappedRight = MessageWrapper.From(
                    MessageType.ImageFrame,
                    rightFrame.ByteBuffer.ToFullArray()
                );

                var fullFrame = MessageConstructor.CreateImageFrame(
                    (uint)combined.Width,
                    (uint)combined.Height,
                    "combined_view",
                    CvUtils.FromMat(combined)
                );

                var wrappedFull = MessageWrapper.From(
                    MessageType.ImageFrame,
                    fullFrame.ByteBuffer.ToFullArray()
                );

                EventBus.Instance.Publish(new MessageWrapperEvent(wrappedLeft));
                EventBus.Instance.Publish(new MessageWrapperEvent(wrappedRight));
                EventBus.Instance.Publish(new MessageWrapperEvent(wrappedFull));

                combined.Dispose();
                leftMat.Dispose();
                rightMat.Dispose();

                await Task.Delay((int)(60 * 1000 / Configuration.FakeCameraSubsystemConfig.FPS), token);
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

        await Shutdown();
    }

    public override Task Shutdown()
    {
        _video?.Release();

        SetOperatingState(SubsystemState.Shutdown);

        return Task.CompletedTask;
    }
}