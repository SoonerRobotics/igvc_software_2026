using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("CameraSubsystem", Disabled = false)]
public class CameraSubsystem : SubsystemBase
{
    // Configuration

    [Config("subsystem.camera.left_path")]
    public static readonly string LeftCameraPath = "/dev/video0";

    [Config("subsystem.camera.right_path")]
    public static readonly string RightCameraPath = "/dev/video2"; // the global shutter cameras are weird and have 2 /video devices per camera

    [Config("subsystem.camera.fps")]
    public static readonly int CameraFps = 12; //TODO is there a reason this is here instead of in Configuration.cs?

    [Config("subsystem.camera.reconnect_delay_ms")]
    public static readonly int ReconnectDelayMs = 2000;

    // Implementation

    private CameraWorker? mLeftWorker;
    private CameraWorker? mRightWorker;

    public override Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        mLeftWorker = new CameraWorker("left", LeftCameraPath, Logger);
        mRightWorker = new CameraWorker("right", RightCameraPath, Logger);

        Subscribe<ConfigChangedEvent>(OnConfigChanged, token);

        _ = Task.Factory.StartNew(
           () => mLeftWorker.RunAsync(token),
           token,
           TaskCreationOptions.LongRunning,
           TaskScheduler.Default
       );

        _ = Task.Factory.StartNew(
            () => mRightWorker.RunAsync(token),
            token,
            TaskCreationOptions.LongRunning,
            TaskScheduler.Default
        );

        SetOperatingState(SubsystemState.Ready);

        return Task.CompletedTask;
    }

    private Task OnConfigChanged(ConfigChangedEvent e, CancellationToken token)
    {
        if (!e.Path.StartsWith("subsystem.camera"))
        {
            return Task.CompletedTask;
        }

        mLeftWorker?.Stop();
        mLeftWorker = new CameraWorker("left", (string)e.Value, Logger);

        mRightWorker?.Stop();
        mRightWorker = new CameraWorker("right", (string)e.Value, Logger);

        return Task.CompletedTask;
    }

    // CameraWorker

    public class CameraWorker(string name, string path, ILogger logger)
    {
        private readonly Lock mFrameLock = new();

        private Mat? mLastFrame;
        private volatile bool mIsConnected;
        private volatile bool mIsStopped = false;

        public bool IsConnected => mIsConnected;

        private readonly ILogger mLogger = logger;

        public Mat? LatestFrame
        {
            get
            {
                lock (mFrameLock)
                {
                    return mLastFrame?.Clone();
                }
            }
        }

        public async Task RunAsync(CancellationToken token)
        {
            while (!token.IsCancellationRequested && !mIsStopped)
            {
                try
                {
                    await ConnectAndCaptureAsync(token);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    mLogger.LogError("Camera {} encountered an error!\n {}", name, ex);
                }

                if (!token.IsCancellationRequested)
                {
                    mIsConnected = false;

                    mLogger.LogWarning("Camera {} reconnecting...", name);

                    await Task.Delay(ReconnectDelayMs, token).ConfigureAwait(false);
                }
            }

            Cleanup();
        }

        private async Task ConnectAndCaptureAsync(CancellationToken token)
        {
            var interval = TimeSpan.FromSeconds(1.0 / CameraFps);
            using var capture = OpenCapture();
            if (capture is null)
            {
                logger.LogError("Failed to open camera: {}", name);
                return;
            }

            capture.Set(VideoCaptureProperties.Fps, CameraFps);
            mIsConnected = true;
            logger.LogInformation("Camera {} connected!", name);

            using var frame = new Mat();
            while (!token.IsCancellationRequested && !mIsStopped)
            {
                var start = DateTime.UtcNow;
                if (!capture.Read(frame) || frame.Empty())
                {
                    logger.LogError("Camera {} disconnected!", name);
                    mIsConnected = false;

                    return;
                }

                lock (mFrameLock)
                {
                    mLastFrame?.Dispose();
                    mLastFrame = frame.Clone();
                }
                BroadcastFrame();

                var elapsed = DateTime.UtcNow - start;
                var remaining = interval - elapsed;
                if (remaining > TimeSpan.Zero)
                {
                    await Task.Delay(remaining, token).ConfigureAwait(false);
                }
                else
                {
                    // we are taking too long to capture frames, low fps
                    // logger.LogDebug("Camera {} has low FPS", name);
                }
            }
        }

        private void BroadcastFrame()
        {
            if (mLastFrame == null)
            {
                return;
            }

            byte[] frameBytes;
            lock (mFrameLock)
            {
                frameBytes = CvUtils.FromMat(mLastFrame);
            }

            var frame = MessageConstructor.CreateImageFrame(1280, 720, name, frameBytes);
            var msg = MessageWrapper.From(MessageType.ImageFrame, frame.ByteBuffer);
            EventBus.Instance.Publish(msg);
        }

        private VideoCapture? OpenCapture()
        {
            try
            {
                // try int if available, else use as path
                var capture = int.TryParse(path, out var idx)
                    ? new VideoCapture(idx)
                    : new VideoCapture(path);
                return capture.IsOpened() ? capture : null;
            }
            catch
            {
                return null;
            }
        }

        public void Cleanup()
        {
            mIsConnected = false;

            lock (mFrameLock)
            {
                mLastFrame?.Dispose();
                mLastFrame = null;
            }
        }

        public void Stop()
        {
            mIsStopped = true;
        }
    }
}