namespace igvc_csharp.External;

using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;

public sealed class SharedProcessManager : IDisposable
{
    private Process? _process;

    public bool IsRunning => _process is { HasExited: false };

    public int? ExitCode => _process?.HasExited == true ? _process.ExitCode : null;

    public event Action<string>? StdOutReceived;
    public event Action<string>? StdErrReceived;
    public event Action<int>? ProcessExited;

    public void Start(string path, string? arguments = null)
    {
        if (IsRunning)
        {
            throw new InvalidOperationException("Process already running");
        }

        _process = new Process
        {
            StartInfo = new ProcessStartInfo
            {
                FileName = path,
                Arguments = arguments ?? string.Empty,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
                CreateNoWindow = true,
            },
            EnableRaisingEvents = true
        };

        _process.OutputDataReceived += (_, e) =>
        {
            if (!string.IsNullOrWhiteSpace(e.Data))
            {
                StdOutReceived?.Invoke(e.Data);
            }
        };

        _process.ErrorDataReceived += (_, e) =>
        {
            if (!string.IsNullOrWhiteSpace(e.Data))
            {
                StdErrReceived?.Invoke(e.Data);
            }
        };

        _process.Exited += (_, _) =>
        {
            try
            {
                ProcessExited?.Invoke(_process!.ExitCode);
            }
            catch
            {
                // ignored
            }
        };

        if (!_process.Start())
        {
            throw new InvalidOperationException("Failed to start process");
        }

        _process.BeginOutputReadLine();
        _process.BeginErrorReadLine();
    }

    public void Stop()
    {
        if (!IsRunning)
        {
            return;
        }

        try
        {
            _process!.Kill();
        }
        catch
        {
            // ignored
        }

        try
        {
            _process!.WaitForExit();
        }
        catch
        {
            // ignored
        }
    }

    public async Task StopAsync(int gracefulTimeoutMs = 1000, CancellationToken token = default)
    {
        if (!IsRunning)
        {
            return;
        }

        try
        {
            // On Linux this usually does nothing, but it's harmless.
            _process!.CloseMainWindow();
        }
        catch
        {
            // ignored
        }

        try
        {
            var exited = await Task.Run(() => _process!.WaitForExit(gracefulTimeoutMs), token);
            if (!exited && !_process!.HasExited)
            {
                _process.Kill();
                _process.WaitForExit();
            }
        }
        catch
        {
            // Best effort shutdown
            Stop();
        }
    }

    public void Dispose()
    {
        Stop();
        _process?.Dispose();
        _process = null;
    }
}