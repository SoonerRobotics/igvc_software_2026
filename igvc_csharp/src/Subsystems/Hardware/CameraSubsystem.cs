using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Core.Hardware;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;
using static igvc_csharp.Subsystems.ChronosSubsystem;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("CameraSubsystem", Disabled = Configuration.UseSimulation)]
public class CameraSubsystem(
    ChronosSubsystem? chronos
) : SubsystemBase
{
    [Config("subsystem.camera.left_shm")]
    public static string LeftShmName = "/camera_left";

    [Config("subsystem.camera.right_shm")]
    public static string RightShmName = "/camera_right";

    [Config("subsystem.camera.reconnect_delay_ms")]
    public static int ReconnectDelayMs = 2000;

    [Config("subsystem.camera.staleness_threshold_ms")]
    public static int StalenessThresholdMs = 5000;

    private CameraWorker? mLeftWorker;
    private CameraWorker? mRightWorker;
    private ProcessManager? mCameraProcess;

    public override async Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        mLeftWorker = new CameraWorker("left", LeftShmName, LeftShmName + "_sem", Logger, chronos);
        mRightWorker = new CameraWorker("right", RightShmName, RightShmName + "_sem", Logger, chronos);

        _ = Task.Factory.StartNew(() => mLeftWorker.RunAsync(token), token, TaskCreationOptions.LongRunning, TaskScheduler.Default);
        _ = Task.Factory.StartNew(() => mRightWorker.RunAsync(token), token, TaskCreationOptions.LongRunning, TaskScheduler.Default);

        var processConfig = new ProcessManagerConfig
        {
            AutoRestart = true,
            RestartDelayMs = 3000,
            CrashThresholdMs = 3000,
            GracefulShutdownTimeoutMs = 3000
        };

        mCameraProcess = new ProcessManager(
            Path.Combine(FileUtils.GetRepositoryRootDirectory(), "igvc_cameras", "build", "igvc_camera"),
            processConfig
        );
        mCameraProcess.LogReceived += OnProcessLog;
        await mCameraProcess.StartAsync(token);

        SetOperatingState(SubsystemState.Ready);
    }

    public override async Task Shutdown()
    {
        mLeftWorker?.Stop();
        mRightWorker?.Stop();

        if (mCameraProcess?.Status == ProcessStatus.Running)
            await mCameraProcess.StopAsync();
    }

    private void OnProcessLog(object? sender, SpdLogStructure log)
    {
        if (log.Message.StartsWith("CAMERA_LEFT_CONNECTED"))
            Logger.LogInformation("Left camera connected");
        else if (log.Message.StartsWith("CAMERA_RIGHT_CONNECTED"))
            Logger.LogInformation("Right camera connected");
        else if (log.Message.StartsWith("CAMERA_LEFT_DISCONNECTED"))
            Logger.LogWarning("Left camera disconnected");
        else if (log.Message.StartsWith("CAMERA_RIGHT_DISCONNECTED"))
            Logger.LogWarning("Right camera disconnected");
        else if (log.Message.StartsWith("SHM_INIT_FAILED"))
        {
            SetOperatingState(SubsystemState.Errored);
            SetError("SHM_INIT_FAILED");
            Logger.LogError("Camera shared memory init failed");
        }
    }

    // ── CameraWorker ──────────────────────────────────────────────────────────

    public class CameraWorker(string name, string shmName, string semName, ILogger logger, ChronosSubsystem? chronos = null)
    {
        private readonly Lock mFrameLock = new();
        private Mat? mLastFrame;
        private volatile bool mIsStopped;

        public Mat? LatestFrame
        {
            get { lock (mFrameLock) { return mLastFrame?.Clone(); } }
        }

        public async Task RunAsync(CancellationToken token)
        {
            while (!token.IsCancellationRequested && !mIsStopped)
            {
                try
                {
                    await ReadLoopAsync(token);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    logger.LogError("Camera {} error: {}", name, ex);
                }

                if (!token.IsCancellationRequested && !mIsStopped)
                {
                    logger.LogWarning("Camera {} reconnecting...", name);
                    await Task.Delay(ReconnectDelayMs, token).ConfigureAwait(false);
                }
            }

            Cleanup();
        }

        private async Task ReadLoopAsync(CancellationToken token)
        {
            using var shm = new CameraFrameSharedMemoryReader(shmName, semName);

            // Wait for the C++ process to create the SHM region
            while (!token.IsCancellationRequested && !mIsStopped && !shm.IsOpen)
            {
                if (shm.TryOpen()) break;
                await Task.Delay(1000, token).ConfigureAwait(false);
            }

            if (!shm.IsOpen) return;

            logger.LogInformation("Camera {} shared memory opened", name);

            uint lastSeq = 0;
            var lastNewFrameAt = DateTime.UtcNow;
            while (!token.IsCancellationRequested && !mIsStopped)
            {
                var result = shm.TryRead(lastSeq, timeoutMs: 150);

                if (result is null)
                {
                    if ((DateTime.UtcNow - lastNewFrameAt).TotalMilliseconds > StalenessThresholdMs)
                    {
                        logger.LogWarning("Camera {} stale — no frames for {}ms", name, StalenessThresholdMs);
                        return; // triggers reconnect loop
                    }

                    await Task.Delay(10, token).ConfigureAwait(false);
                    continue;
                }

                var (header, pixels) = result.Value;
                lastSeq = header.SequenceNum;
                lastNewFrameAt = DateTime.UtcNow;

                var mat = DecodeBgrFrame(pixels, CameraFrameSharedMemoryReader.FrameWidth,
                                                  CameraFrameSharedMemoryReader.FrameHeight);
                Mat? old;
                lock (mFrameLock)
                {
                    old = mLastFrame;
                    mLastFrame = mat;
                }
                old?.Dispose();

                // Handle the new frame
                BroadcastFrame();
                chronos?.WriteVideoFrame(
                    name == "left" ? CameraId.Left : CameraId.Right,
                    mat
                );

                await Task.Delay(5, token).ConfigureAwait(false);
            }
        }

        private void BroadcastFrame()
        {
            byte[] frameBytes;
            lock (mFrameLock)
            {
                if (mLastFrame == null) return;
                frameBytes = CvUtils.FromMat(mLastFrame);
            }

            var frame = MessageConstructor.CreateImageFrame(
                CameraFrameSharedMemoryReader.FrameWidth,
                CameraFrameSharedMemoryReader.FrameHeight,
                name, frameBytes);
            var msg = MessageWrapper.From(MessageType.ImageFrame, frame.ByteBuffer);
            EventBus.Instance.Publish(msg);
        }

        private static Mat DecodeBgrFrame(byte[] bgrPixels, int width, int height)
        {
            var mat = new Mat(height, width, MatType.CV_8UC3);
            unsafe
            {
                fixed (byte* src = bgrPixels)
                    Buffer.MemoryCopy(src, mat.DataPointer, bgrPixels.Length, bgrPixels.Length);
            }
            return mat;
        }

        public void Stop() => mIsStopped = true;

        private void Cleanup()
        {
            lock (mFrameLock)
            {
                mLastFrame?.Dispose();
                mLastFrame = null;
            }
        }
    }
}