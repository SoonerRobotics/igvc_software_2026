using igvc_csharp.Core;
using igvc_csharp.Core.Hardware;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;
using static igvc_csharp.Subsystems.ChronosSubsystem;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("ZedSubsystem", Disabled = false)]
public class ZedSubsystem(
    ChronosSubsystem? chronos
) : SubsystemBase
{
    private Task? _frameTask;
    private CancellationTokenSource? _cts;
    private ProcessManager? _zedProcessManager;

    private readonly SubsystemProperty<uint> _pLastSequence = new("frame_sequence", 0);
    private readonly SubsystemProperty<double> _pFps = new("fps", 0.0);
    private readonly SubsystemProperty<string> _pSerialNumber = new("serial_number");

    private readonly object _frameLock = new();

    public Mat? LatestFrame { get; private set; }
    public Mat? CloneLatestFrame()
    {
        lock (_frameLock)
            return LatestFrame?.Clone();
    }

    private ZedDepthSharedMemoryChannel? _depthChannel;
    private uint _nextRequestId = 1;
    private readonly SemaphoreSlim _depthSem = new(1, 1);

    public override async Task Init(CancellationToken token)
    {
        _cts = CancellationTokenSource.CreateLinkedTokenSource(token);

        _depthChannel = new ZedDepthSharedMemoryChannel();

        _frameTask = Task.Run(() => FrameLoop(_cts.Token), _cts.Token);
        SetOperatingState(SubsystemState.Idle);

        var zedConfig = new ProcessManagerConfig
        {
            AutoRestart = true,
            RestartDelayMs = 3000,
            CrashThresholdMs = 3000,
            GracefulShutdownTimeoutMs = 3000
        };

        _zedProcessManager = new ProcessManager(
            Path.Combine(FileUtils.GetRepositoryRootDirectory(), "igvc_zed", "build", "igvc_zed"),
            zedConfig
        );
        _zedProcessManager.LogReceived += OnLogReceived;
        await _zedProcessManager.StartAsync(token);
    }

    public override async Task Shutdown()
    {
        if (_zedProcessManager?.Status == ProcessStatus.Running)
            await _zedProcessManager.StopAsync();

        _cts?.Cancel();

        if (_frameTask is not null)
            await _frameTask.ConfigureAwait(false);

        lock (_frameLock)
        {
            LatestFrame?.Dispose();
            LatestFrame = null;
        }

        _depthChannel?.Dispose();
    }

    private void OnLogReceived(object? sender, SpdLogStructure log)
    {
        if (log.Message.StartsWith("ZED_STARTING"))
        {
            SetOperatingState(SubsystemState.Starting);
            return;
        }

        if (log.Message.StartsWith("ZED_CONNECTION_FAILED"))
        {
            SetOperatingState(SubsystemState.Errored);
            SetError("ZED_CONNECTION_FAILED");
            return;
        }

        if (log.Message.StartsWith("ZED_CONNECTED_SN_"))
        {
            var sn = log.Message.Replace("ZED_CONNECTED_SN_", "");
            _pSerialNumber.Set(sn);
            Logger.LogInformation("ZED 2i connected (SN={})", sn);
            return;
        }

        if (log.Message.StartsWith("ZED_DISCONNECTED"))
        {
            SetOperatingState(SubsystemState.Errored);
            _pSerialNumber.Set(null);
            ClearError();
            return;
        }

        if (log.Message.StartsWith("FRAME_SHM_INIT_FAILED") ||
            log.Message.StartsWith("DEPTH_SHM_INIT_FAILED"))
        {
            SetOperatingState(SubsystemState.Errored);
            SetError(log.Message);
            Logger.LogError("ZED shared memory init failed: {}", log.Message);
        }
    }

    private async Task FrameLoop(CancellationToken token)
    {
        using var shm = new ZedFrameSharedMemoryReader();

        while (!token.IsCancellationRequested)
        {
            shm.Close();
            while (!token.IsCancellationRequested && !shm.IsOpen)
            {
                if (shm.TryOpen()) break;
                await Task.Delay(1000, token).ConfigureAwait(false);
            }

            if (!shm.IsOpen) break;

            if (!(_depthChannel?.IsOpen ?? false))
                _depthChannel?.TryOpen();

            uint lastSeq = 0;
            var lastNewFrameAt = DateTime.UtcNow;
            const int stalenessThresholdMs = 5000;

            var fpsWindow = new Queue<DateTime>();
            const int fpsSampleCount = 30;
            try
            {
                while (!token.IsCancellationRequested)
                {
                    var result = shm.TryRead(lastSeq, timeoutMs: 150);

                    if (result is null)
                    {
                        if ((DateTime.UtcNow - lastNewFrameAt).TotalMilliseconds > stalenessThresholdMs)
                        {
                            SetError("ZED_NO_FRAMES");
                            break;
                        }

                        await Task.Delay(10, token).ConfigureAwait(false);
                        continue;
                    }

                    var (header, pixels) = result.Value;
                    lastSeq = header.SequenceNum;
                    lastNewFrameAt = DateTime.UtcNow;

                    var mat = DecodeBgraFrame(pixels, ZedFrameSharedMemoryReader.FrameWidth,
                                                      ZedFrameSharedMemoryReader.FrameHeight);

                    Mat? old;
                    lock (_frameLock)
                    {
                        old = LatestFrame;
                        LatestFrame = mat;
                    }
                    old?.Dispose();

                    // TODO: Handle frame
                    BroadcastFrame();
                    chronos?.WriteVideoFrame(
                        CameraId.Zed2i,
                        mat
                    );

                    _pLastSequence.Set(header.SequenceNum);

                    fpsWindow.Enqueue(DateTime.UtcNow);
                    if (fpsWindow.Count > fpsSampleCount) fpsWindow.Dequeue();
                    if (fpsWindow.Count >= 2)
                    {
                        var span = (fpsWindow.Last() - fpsWindow.First()).TotalSeconds;
                        _pFps.Set(Math.Round((fpsWindow.Count - 1) / span, 1));
                    }

                    SetOperatingState(SubsystemState.Operating);
                    SetError(string.Empty);

                    await Task.Delay(5, token).ConfigureAwait(false);
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "Unexpected error in ZED frame loop, reconnecting...");
                await Task.Delay(1000, token).ConfigureAwait(false);
            }
        }

        Logger.LogInformation("ZED frame loop stopped");
    }

    private void BroadcastFrame()
    {
        if (LatestFrame == null)
        {
            return;
        }

        byte[] frameBytes;
        lock (_frameLock)
        {
            frameBytes = CvUtils.FromMat(LatestFrame);
        }

        var frame = MessageConstructor.CreateImageFrame(1280, 720, "zed", frameBytes);
        var msg = MessageWrapper.From(MessageType.ImageFrame, frame.ByteBuffer);
        EventBus.Instance.Publish(msg);
    }

    public async Task<ZedDepthResponse?> RequestDepthAsync(int pixelX, int pixelY, int timeoutMs = 500)
    {
        if (_depthChannel is null || !_depthChannel.IsOpen)
        {
            Logger.LogWarning("Depth channel not open — is the ZED process running?");
            return null;
        }

        if (pixelX < 0 || pixelX >= ZedFrameSharedMemoryReader.FrameWidth ||
            pixelY < 0 || pixelY >= ZedFrameSharedMemoryReader.FrameHeight)
        {
            Logger.LogWarning("RequestDepth out of bounds: ({X}, {Y})", pixelX, pixelY);
            return null;
        }

        await _depthSem.WaitAsync().ConfigureAwait(false);
        try
        {
            var id = _nextRequestId++;
            return await Task.Run(() => _depthChannel.RequestDepth(id, pixelX, pixelY, timeoutMs))
                             .ConfigureAwait(false);
        }
        finally
        {
            _depthSem.Release();
        }
    }

    private static Mat DecodeBgraFrame(byte[] bgraPixels, int width, int height)
    {
        var mat = new Mat(height, width, MatType.CV_8UC4);
        unsafe
        {
            fixed (byte* src = bgraPixels)
            {
                Buffer.MemoryCopy(src, mat.DataPointer, bgraPixels.Length, bgraPixels.Length);
            }
        }
        return mat;
    }
}