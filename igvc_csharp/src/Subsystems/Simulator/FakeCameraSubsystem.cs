using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;
using Silk.NET.Core.Native;

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

                Mat mat = new();
                if (_video == null || !_video.Read(mat))
                {
                    EOF = true;
                    continue;
                }

                var frameBytes = CvUtils.FromMat(mat);
                var newFrame = MessageConstructor.CreateImageFrame(
                    (uint)mat.Width,
                    (uint)mat.Height,
                    "front_view",
                    frameBytes
                );

                var wrappedFrame = MessageWrapper.From(
                    MessageType.ImageFrame,
                    newFrame.ByteBuffer.ToFullArray()
                );

                EventBus.Instance.Publish(new MessageWrapperEvent(wrappedFrame));

                mat.Dispose();

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