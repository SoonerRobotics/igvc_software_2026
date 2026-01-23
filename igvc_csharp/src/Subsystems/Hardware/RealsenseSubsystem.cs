using igvc_csharp.Core;
using igvc_csharp.External;
using igvc_csharp.Utils;
using Microsoft.Extensions.Logging;
using System.Runtime.InteropServices;
using igvc_csharp.Utils.Messages;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("RealsenseSubsystem", DependsOn = [], Disabled = false)]
public class RealsenseSubsystem : SubsystemBase
{
    private readonly SharedProcessManager _processManager = new();

    private CancellationTokenSource? _runCts;
    private string? _executablePath;

    private const string ShmName = "realsense_frames";

    private static readonly TimeSpan RestartDelay = TimeSpan.FromSeconds(2);
    private static readonly TimeSpan MaxBackoff = TimeSpan.FromSeconds(10);
    private static readonly TimeSpan BackoffResetUptime = TimeSpan.FromSeconds(15);

    private static readonly TimeSpan ShmStaleThreshold = TimeSpan.FromMilliseconds(500);
    private static readonly TimeSpan ShmDeadThreshold = TimeSpan.FromSeconds(2);

    private TimeSpan _currentBackoff = RestartDelay;
    private DateTime _lastStartUtc = DateTime.MinValue;

    private SharedMemoryChannel? _shm;
    private long _lastSequence = -1;

    private byte[]? _rgbBuffer;
    private byte[]? _depthBuffer;

    public override Task Init(CancellationToken token)
    {
        _processManager.StdOutReceived += OnStdOut;
        _processManager.StdErrReceived += OnStdErrStructured;
        _processManager.ProcessExited += OnProcessExit;

        _runCts = CancellationTokenSource.CreateLinkedTokenSource(token);
        _ = Task.Run(() => InitializeAndRunAsync(_runCts.Token), _runCts.Token);

        return Task.CompletedTask;
    }

    private void OnStdOut(string line)
    {
        Logger.LogDebug("[RealSense:stdout] {Line}", line);
    }

    private void OnStdErrStructured(string line)
    {
        var parts = line.Split('|', 3);
        if (parts.Length != 3)
        {
            Logger.LogError("[RealSense] {Line}", line);
            return;
        }

        var level = parts[0];
        var category = parts[1];
        var message = parts[2];

        switch (level)
        {
            case "INFO":
                Logger.LogInformation("RealSense [{Category}] {Message}", category, message);
                break;

            case "WARN":
                Logger.LogWarning("RealSense [{Category}] {Message}", category, message);
                break;

            case "ERR":
                Logger.LogError("RealSense [{Category}] {Message}", category, message);
                break;

            default:
                Logger.LogError("[RealSense] {Line}", line);
                break;
        }
    }

    private void OnProcessExit(int code)
    {
        Logger.LogWarning("RealSense process exited with code {Code}", code);
    }

    private async Task InitializeAndRunAsync(CancellationToken token)
    {
        var cppDir = Path.Join(FileUtils.GetRepositoryRootDirectory(), "igvc_cpp");
        var buildDir = Path.Join(cppDir, "build");

        if (!Directory.Exists(cppDir))
        {
            Logger.LogCritical("Failed to find igvc_cpp directory at {Path}", cppDir);
            SetState(SubsystemState.Fatal);
            return;
        }

        if (!await BuildCppAsync(cppDir, buildDir, token))
        {
            SetState(SubsystemState.Fatal);
            return;
        }

        _executablePath = Path.Join(buildDir, "realsense");
        if (!File.Exists(_executablePath))
        {
            Logger.LogCritical("Executable not found at {Path}", _executablePath);
            SetState(SubsystemState.Fatal);
            return;
        }

        SetState(SubsystemState.Operating);
        while (!token.IsCancellationRequested)
        {
            try
            {
                CleanupSharedMemory(ShmName);

                Logger.LogInformation("Starting RealSense process");
                _lastStartUtc = DateTime.UtcNow;
                _processManager.Start(_executablePath);

                AttachSharedMemory();

                while (_processManager.IsRunning && !token.IsCancellationRequested)
                {
                    MonitorSharedMemory();
                    await Task.Delay(50, token);
                }

                DetachSharedMemory();

                var uptime = DateTime.UtcNow - _lastStartUtc;
                if (uptime >= BackoffResetUptime)
                {
                    _currentBackoff = RestartDelay;
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "Exception in RealSense supervisor loop");
            }

            if (token.IsCancellationRequested)
                break;

            Logger.LogWarning("RealSense stopped. Restarting in {Delay}s", _currentBackoff.TotalSeconds);

            await Task.Delay(_currentBackoff, token);

            _currentBackoff = TimeSpan.FromSeconds(Math.Min(_currentBackoff.TotalSeconds * 2, MaxBackoff.TotalSeconds));
        }

        Logger.LogInformation("RealSense supervisor exiting");
    }

    private void AttachSharedMemory()
    {
        _shm = new SharedMemoryChannel(ShmName);

        var header = _shm.ReadHeader();

        _rgbBuffer = new byte[header.RgbSizeBytes];
        _depthBuffer = new byte[header.DepthSizeBytes];

        _lastSequence = header.Sequence;

        Logger.LogInformation("Attached to shared memory ({Width}x{Height})", header.Width, header.Height);
    }

    private void DetachSharedMemory()
    {
        _shm?.Dispose();
        _shm = null;

        _rgbBuffer = null;
        _depthBuffer = null;
        _lastSequence = -1;
    }

    private void MonitorSharedMemory()
    {
        if (_shm == null)
        {
            return;
        }

        var header = _shm.ReadHeader();
        var writeTime = DateTime.UnixEpoch.AddTicks(header.LastWriteTicks);
        var age = DateTime.UtcNow - writeTime;
        if (header.Sequence == _lastSequence)
        {
            if (age > ShmDeadThreshold)
            {
                Logger.LogError("Shared memory stalled — forcing restart");
                _processManager.Stop();
            }

            return;
        }

        _lastSequence = header.Sequence;
        if (age > ShmStaleThreshold)
        {
            Logger.LogWarning("Shared memory data stale ({Age} ms)", age.TotalMilliseconds);
        }

        // Read frame payloads
        var rgbOffset = Marshal.SizeOf<FrameHeader>();
        var depthOffset = rgbOffset + _rgbBuffer!.Length;

        _shm.ReadBytes(rgbOffset, _rgbBuffer!);
        _shm.ReadBytes(depthOffset, _depthBuffer!);

        // At this point:
        var imageFrame = MessageConstructor.CreateImageFrame(
            (uint)header.Width, 
            (uint)header.Height, 
            "front",
            _rgbBuffer
            );
        var depthFrame = MessageConstructor.CreateDepthFrame(
            (uint)header.Width,
            (uint)header.Height,
            "front",
            _depthBuffer
            );

        var imageWrapper = MessageWrapper.From(MessageType.ImageFrame, imageFrame.ByteBuffer);
        var depthWrapper = MessageWrapper.From(MessageType.DepthFrame, depthFrame.ByteBuffer);
        
        EventBus.Instance.Publish(imageWrapper.Event());
        EventBus.Instance.Publish(depthWrapper.Event());
    }

    private async Task<bool> BuildCppAsync(string cppDir, string buildDir, CancellationToken token)
    {
        Logger.LogInformation("Building RealSense C++ project");

        var result = await CMakeBuildRunner.BuildAsync(
            cppDir,
            buildDir,
            "Release",
            token);

        if (result.Result != CMakeBuildRunner.BuildResult.Success)
        {
            Logger.LogCritical("CMake build failed:\n{StdErr}", result.StdErr);
            return false;
        }

        return true;
    }

    private void CleanupSharedMemory(string name)
    {
        TryDelete($"/dev/shm/{name}");
        TryDelete($"/dev/shm/sem.{name}_sem");
    }

    private void TryDelete(string path)
    {
        try
        {
            if (!File.Exists(path))
            {
                return;
            }

            File.Delete(path);
            Logger.LogInformation("Deleted stale shared memory artifact: {Path}", path);
        }
        catch (Exception ex)
        {
            Logger.LogWarning(ex, "Failed to cleanup {Path}", path);
        }
    }

    public override Task Restart()
    {
        SetState(SubsystemState.Ready);

        _runCts?.Cancel();
        _processManager.Stop();

        _currentBackoff = RestartDelay;
        _runCts = new CancellationTokenSource();

        _ = Task.Run(() => InitializeAndRunAsync(_runCts.Token));
        return Task.CompletedTask;
    }

    public override Task Shutdown()
    {
        SetState(SubsystemState.ShuttingDown);

        _runCts?.Cancel();
        _processManager.Stop();
        DetachSharedMemory();

        SetState(SubsystemState.Shutdown);
        return Task.CompletedTask;
    }
}