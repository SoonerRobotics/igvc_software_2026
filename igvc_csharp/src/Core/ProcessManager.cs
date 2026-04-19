using System.Diagnostics;
using System.Text.Json;
using System.Text.Json.Nodes;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Core;

public enum ProcessStatus
{
    Stopped,
    Starting,
    Running,
    Restarting,
    Failed,
}

public class StatusChangedEventArgs(ProcessStatus previous, ProcessStatus current) : EventArgs
{
    public ProcessStatus Previous { get; } = previous;
    public ProcessStatus Current { get; } = current;
}

public sealed class ProcessManager : IAsyncDisposable
{
    private static ILogger Logger = Logging.From<ProcessManager>();

    private readonly string _exePath;
    private readonly ProcessManagerConfig _config;

    private Process? _process;
    private CancellationTokenSource? _cts;
    private Task? _monitorTask;
    private int _restartCount;
    private int _jsonParseErrorCount;
    private DateTime _processStartTime;
    private readonly SemaphoreSlim _lock = new(1, 1);

    private ProcessStatus _status = ProcessStatus.Stopped;

    public event EventHandler<object>? JsonReceived;
    public event EventHandler<StatusChangedEventArgs>? StatusChanged;
    public event EventHandler<string>? RawOutputReceived;
    public event EventHandler<string>? ErrorOutputReceived;

    public ProcessStatus Status => _status;
    public int RestartCount => _restartCount;
    public string ExePath => _exePath;

    public ProcessManager(string exePath, ProcessManagerConfig? config = null)
    {
        if (string.IsNullOrWhiteSpace(exePath))
        {
            throw new ArgumentException("Executable path must not be empty.", nameof(exePath));
        }

        _exePath = exePath;
        _config = config ?? new ProcessManagerConfig();
    }

    public async Task StartAsync(CancellationToken cancellationToken = default)
    {
        await _lock.WaitAsync(cancellationToken);
        try
        {
            if (_status is ProcessStatus.Running or ProcessStatus.Starting)
            {
                throw new InvalidOperationException("Process is already running.");
            }

            _cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
            _restartCount = 0;

            SetStatus(ProcessStatus.Starting);
            await LaunchProcessAsync(_cts.Token);

            _monitorTask = MonitorLoopAsync(_cts.Token);
        }
        finally
        {
            _lock.Release();
        }
    }

    public async Task StopAsync()
    {
        await _lock.WaitAsync();
        try
        {
            if (_status is ProcessStatus.Stopped or ProcessStatus.Failed)
            {
                return;
            }

            await _cts!.CancelAsync();
            await KillProcessAsync();
        }
        finally
        {
            _lock.Release();
        }

        if (_monitorTask is not null)
        {
            try
            {
                await _monitorTask;
            }
            catch (OperationCanceledException)
            {

            }
        }

        SetStatus(ProcessStatus.Stopped);
    }

    private async Task MonitorLoopAsync(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            try
            {
                await _process!.WaitForExitAsync(ct);
            }
            catch (OperationCanceledException)
            {
                break;
            }

            if (ct.IsCancellationRequested)
            {
                break;
            }

            int exitCode = _process.ExitCode;
            double runtimeMs = (DateTime.UtcNow - _processStartTime).TotalMilliseconds;
            bool isCrash = runtimeMs < _config.CrashThresholdMs;

            Logger.LogError("[ProcessManager] Process exited (code={}, runtime={}. crash={})", exitCode, runtimeMs, isCrash);
            if (!_config.AutoRestart)
            {
                SetStatus(ProcessStatus.Stopped);
                break;
            }

            if (_config.MaxRestartAttempts >= 0 && _restartCount >= _config.MaxRestartAttempts)
            {
                Logger.LogError("[ProcessManager] Unable to restart, max restart attempts ({}) reached", _config.MaxRestartAttempts);
                SetStatus(ProcessStatus.Failed);
                break;
            }

            SetStatus(ProcessStatus.Restarting);
            _restartCount++;

            Logger.LogError(
                "[ProcessManager] Restarting in {}ms (attempts {}/{})",
                _config.RestartDelayMs,
                _restartCount,
                _config.MaxRestartAttempts < 0 ? "inf" : _config.MaxRestartAttempts
            );

            try
            {
                await Task.Delay(_config.RestartDelayMs, ct);
                await LaunchProcessAsync(ct);
                SetStatus(ProcessStatus.Running);
            }
            catch (OperationCanceledException)
            {
                break;
            }
        }
    }

    private Task LaunchProcessAsync(CancellationToken ct)
    {
        _process?.Dispose();

        var psi = new ProcessStartInfo(_exePath, _config.Arguments)
        {
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = true,
        };

        _process = new Process { StartInfo = psi, EnableRaisingEvents = true };

        _process.OutputDataReceived += OnOutputDataReceived;
        _process.ErrorDataReceived += OnErrorDataReceived;

        _process.Start();
        _processStartTime = DateTime.UtcNow;

        _process.BeginOutputReadLine();
        _process.BeginErrorReadLine();

        SetStatus(ProcessStatus.Running);

        Logger.LogInformation("[ProcessManager] Started PID {} -> {}", _process.Id, _exePath);
        return Task.CompletedTask;
    }

    private void OnOutputDataReceived(object sender, DataReceivedEventArgs e)
    {
        if (e.Data is null)
        {
            return;
        }

        RawOutputReceived?.Invoke(this, e.Data);
        TryParseAndEmitJson(e.Data);
    }

    private void OnErrorDataReceived(object sender, DataReceivedEventArgs e)
    {
        if (e.Data is null)
        {
            return;
        }

        ErrorOutputReceived?.Invoke(this, e.Data);
    }

    private void TryParseAndEmitJson(string line)
    {
        var trimmed = line.TrimStart();
        Logger.LogTrace("[ProcessManager] {}", trimmed);
    }

    private async Task KillProcessAsync()
    {
        if (_process is null || _process.HasExited)
        {
            return;
        }

        try
        {
            _process.CloseMainWindow();

            var gracefulExit = await Task.WhenAny(
                _process.WaitForExitAsync(),
                Task.Delay(_config.GracefulShutdownTimeoutMs)
            );

            if (gracefulExit != _process.WaitForExitAsync())
            {
                _process.Kill(entireProcessTree: true);
                Logger.LogWarning("[ProcessManager] Process killed (graceful shutdown timed out)");
            }
        }
        catch (InvalidOperationException)
        {

        }
    }

    private void SetStatus(ProcessStatus next)
    {
        if (_status == next)
        {
            return;
        }

        var prev = _status;
        _status = next;
        StatusChanged?.Invoke(this, new StatusChangedEventArgs(prev, next));
    }

    public async ValueTask DisposeAsync()
    {
        await StopAsync();
        _process?.Dispose();
        _cts?.Dispose();
        _lock.Dispose();
    }
}