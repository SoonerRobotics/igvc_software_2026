namespace igvc_csharp.External;

using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

public static class CMakeBuildRunner
{
    public enum BuildResult
    {
        Success,
        ConfigureFailed,
        BuildFailed,
        Cancelled
    }

    public sealed class BuildOutput
    {
        public BuildResult Result;
        public int ExitCode;
        public string StdOut = "";
        public string StdErr = "";
    }
    
    public static async Task<BuildOutput> BuildAsync(
        string sourceDir,
        string buildDir,
        string buildType = "Release",
        CancellationToken cancellationToken = default)
    {
        Directory.CreateDirectory(buildDir);

        var output = new BuildOutput();
        var stdout = new StringBuilder();
        var stderr = new StringBuilder();

        // 1️⃣ Configure
        var configureResult = await RunProcessAsync(
            "cmake",
            $"-S \"{sourceDir}\" -B \"{buildDir}\" -DCMAKE_BUILD_TYPE={buildType}",
            stdout,
            stderr,
            cancellationToken);

        if (configureResult != 0)
        {
            output.Result = BuildResult.ConfigureFailed;
            output.ExitCode = configureResult;
            output.StdOut = stdout.ToString();
            output.StdErr = stderr.ToString();
            return output;
        }

        var buildResult = await RunProcessAsync(
            "cmake",
            $"--build \"{buildDir}\"",
            stdout,
            stderr,
            cancellationToken);

        output.ExitCode = buildResult;
        output.StdOut = stdout.ToString();
        output.StdErr = stderr.ToString();

        if (cancellationToken.IsCancellationRequested)
        {
            output.Result = BuildResult.Cancelled;
        }
        else if (buildResult != 0)
        {
            output.Result = BuildResult.BuildFailed;
        }
        else
        {
            output.Result = BuildResult.Success;
        }

        return output;
    }
    
    private static Task<int> RunProcessAsync(
        string fileName,
        string arguments,
        StringBuilder stdout,
        StringBuilder stderr,
        CancellationToken cancellationToken)
    {
        var tcs = new TaskCompletionSource<int>();

        var process = new Process
        {
            StartInfo = new ProcessStartInfo
            {
                FileName = fileName,
                Arguments = arguments,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
                CreateNoWindow = true
            },
            EnableRaisingEvents = true
        };

        process.OutputDataReceived += (_, e) =>
        {
            if (e.Data != null)
                stdout.AppendLine(e.Data);
        };

        process.ErrorDataReceived += (_, e) =>
        {
            if (e.Data != null)
                stderr.AppendLine(e.Data);
        };

        process.Exited += (_, _) =>
        {
            tcs.TrySetResult(process.ExitCode);
            process.Dispose();
        };

        if (!process.Start())
            throw new InvalidOperationException("Failed to start process");

        process.BeginOutputReadLine();
        process.BeginErrorReadLine();

        cancellationToken.Register(() =>
        {
            try
            {
                if (!process.HasExited)
                    process.Kill();
            }
            catch { /* ignored */ }

            tcs.TrySetCanceled();
        });

        return tcs.Task;
    }
}
