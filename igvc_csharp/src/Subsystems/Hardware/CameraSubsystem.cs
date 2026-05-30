using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Core.Hardware;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;
using System.Runtime.InteropServices;
using static igvc_csharp.Subsystems.ChronosSubsystem;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("CameraSubsystem", Disabled = Configuration.UseSimulation)]
public class CameraSubsystem(
    ChronosSubsystem? chronos
) : SubsystemBase
{
    [Config("subsystem.camera.center_shm")]
    public static string CenterShmName = "/camera_center";

    [Config("subsystem.camera.reconnect_delay_ms")]
    public static int ReconnectDelayMs = 2000;

    [Config("subsystem.camera.staleness_threshold_ms")]
    public static int StalenessThresholdMs = 5000;

    [Config("subsystem.camera.fps")]
    public static uint DefaultFps = 20;

    private CameraWorker? mCenterWorker;
    private CameraCommandShmWriter? mCenterCmd;
    private ProcessManager? mCameraProcess;

    public override async Task Init(CancellationToken token)
    {
        SetOperatingState(SubsystemState.Starting);

        mCenterWorker = new CameraWorker("center", CenterShmName, CenterShmName + "_sem", Logger, chronos);
        mCenterCmd = new CameraCommandShmWriter("/camera_cmd_center");

        _ = Task.Factory.StartNew(() => mCenterWorker.RunAsync(token), token, TaskCreationOptions.LongRunning, TaskScheduler.Default);

        Subscribe<ConfigChangedEvent>(OnConfigChanged, token);

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

    public Task OnConfigChanged(ConfigChangedEvent e, CancellationToken token)
    {
        if (e.Path == "subsystem.camera.fps")
        {
            SetCameraProperty(CameraProperty.Fps, e.Value as uint? ?? DefaultFps);
        }

        return Task.CompletedTask;
    }

    public override async Task Shutdown()
    {
        mCenterWorker?.Stop();
        mCenterCmd?.Dispose();

        if (mCameraProcess?.Status == ProcessStatus.Running)
            await mCameraProcess.StopAsync();
    }

    private void OnProcessLog(object? sender, SpdLogStructure log)
    {
        if (log.Message.StartsWith("CAMERA_CENTER_CONNECTED"))
            Logger.LogInformation("Center camera connected");
        else if (log.Message.StartsWith("CAMERA_CENTER_DISCONNECTED"))
            Logger.LogWarning("Center camera disconnected");
        else if (log.Message.StartsWith("CAMERA_CENTER_PROPS_APPLIED"))
            Logger.LogInformation("Center camera properties applied");
        else if (log.Message.StartsWith("SHM_INIT_FAILED"))
        {
            SetOperatingState(SubsystemState.Errored);
            SetError("SHM_INIT_FAILED");
            Logger.LogError("Camera shared memory init failed");
        }
    }

    public void SetCameraProperty(CameraProperty prop, uint value)
    {
        mCenterCmd?.Set(prop, value);
        Logger.LogInformation("Camera Center {Prop} set to {Value}", prop, value);
    }

    public enum CameraProperty { Fps, Width, Height, Fourcc }

    public sealed unsafe class CameraCommandShmWriter : IDisposable
    {
        public const uint FourccMjpg = 0x47504A4D;
        public const uint FourccYuy2 = 0x32595559;
        public const uint FourccH264 = 0x34363248;

        private const int BlockSize = 20;

        private static class Posix
        {
            private const string Libc = "libc";

            [DllImport(Libc, SetLastError = true)]
            public static extern int shm_open(string name, int oflag, uint mode);

            [DllImport(Libc, SetLastError = true)]
            public static extern int shm_unlink(string name);

            [DllImport(Libc, SetLastError = true)]
            public static extern int ftruncate(int fd, long length);

            [DllImport(Libc, SetLastError = true)]
            public static extern IntPtr mmap(IntPtr addr, nuint length, int prot,
                                             int flags, int fd, long offset);

            [DllImport(Libc, SetLastError = true)]
            public static extern int munmap(IntPtr addr, nuint length);

            [DllImport(Libc, SetLastError = true)]
            public static extern int close(int fd);

            public const int O_CREAT = 0x40;
            public const int O_RDWR = 0x02;
            public const int PROT_READ = 0x1;
            public const int PROT_WRITE = 0x2;
            public const int MAP_SHARED = 0x01;

            public static readonly IntPtr MAP_FAILED = new(-1);
        }

        private readonly string _shmName;
        private readonly IntPtr _ptr;
        private readonly object _lock = new();

        private uint _version = 0;
        private uint _width = 640;
        private uint _height = 480;
        private uint _fourcc = FourccMjpg;

        public CameraCommandShmWriter(string shmName)
        {
            _shmName = shmName;

            int fd = Posix.shm_open(shmName, Posix.O_CREAT | Posix.O_RDWR, 0b110_110_110);
            if (fd < 0)
                throw new IOException($"shm_open({shmName}) failed: errno {Marshal.GetLastPInvokeError()}");

            try
            {
                if (Posix.ftruncate(fd, BlockSize) < 0)
                    throw new IOException($"ftruncate({shmName}) failed: errno {Marshal.GetLastPInvokeError()}");

                _ptr = Posix.mmap(IntPtr.Zero, BlockSize,
                                  Posix.PROT_READ | Posix.PROT_WRITE,
                                  Posix.MAP_SHARED, fd, 0);

                if (_ptr == Posix.MAP_FAILED)
                    throw new IOException($"mmap({shmName}) failed: errno {Marshal.GetLastPInvokeError()}");
            }
            finally
            {
                Posix.close(fd);
            }

            Flush();
        }

        public void Set(CameraProperty prop, uint value)
        {
            lock (_lock)
            {
                Apply(prop, value);
                _version++;
                Flush();
            }
        }

        public void SetMany(IEnumerable<(CameraProperty Prop, uint Value)> props)
        {
            lock (_lock)
            {
                foreach (var (prop, value) in props)
                    Apply(prop, value);
                _version++;
                Flush();
            }
        }

        private void Apply(CameraProperty prop, uint value)
        {
            switch (prop)
            {
                case CameraProperty.Fps: DefaultFps = value; break;
                case CameraProperty.Width: _width = value; break;
                case CameraProperty.Height: _height = value; break;
                case CameraProperty.Fourcc: _fourcc = value; break;
            }
        }

        private void Flush()
        {
            uint* p = (uint*)_ptr;
            p[0] = _version;
            p[1] = DefaultFps;
            p[2] = _width;
            p[3] = _height;
            p[4] = _fourcc;
        }

        public void Dispose()
        {
            if (_ptr != IntPtr.Zero && _ptr != Posix.MAP_FAILED)
                Posix.munmap(_ptr, BlockSize);
        }
    }

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
                        return;
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

                BroadcastFrame();
                chronos?.WriteVideoFrame(CameraId.Center, mat);

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