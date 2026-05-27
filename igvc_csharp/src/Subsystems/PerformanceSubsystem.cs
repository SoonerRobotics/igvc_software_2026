using System.Threading.Channels;
using igvc_csharp.Core;
using igvc_csharp.Core.Hardware;
using igvc_csharp.Events;
using igvc_csharp.Subsystems;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using igvc_csharp.Yolo;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

[Subsystem("PerformanceSubsystem", Disabled = true)]
public class PerformanceSubsystem(
    ChronosSubsystem chronos
) : SubsystemBase
{
    public SubsystemProperty<float> RamUsage = new SubsystemProperty<float>("RamUsage", 0);
    public SubsystemProperty<float> CpuUsage = new SubsystemProperty<float>("CpuUsage", 0);
    public SubsystemProperty<float> GpuUsage = new SubsystemProperty<float>("GpuUsage", 0);

    public override Task Init(CancellationToken token)
    {
        Task.Run(() => MonitorLoop(token), token);
        return Task.CompletedTask;
    }

    private async Task MonitorLoop(CancellationToken token)
    {
        var (prevIdle, prevTotal) = await ReadCpuStatsAsync();

        while (!token.IsCancellationRequested)
        {
            try
            {
                await Task.Delay(1000, token);

                // --- RAM ---
                float ramPercent = await ReadRamUsageAsync();
                RamUsage.Set(ramPercent);

                // --- CPU ---
                var (idle, total) = await ReadCpuStatsAsync();
                long diffIdle = idle - prevIdle;
                long diffTotal = total - prevTotal;
                float cpuPercent = diffTotal > 0
                    ? (1f - (float)diffIdle / diffTotal) * 100f
                    : 0f;
                CpuUsage.Set(cpuPercent);
                prevIdle = idle;
                prevTotal = total;

                // --- GPU (Jetson) ---
                float gpuPercent = await ReadJetsonGpuUsageAsync();
                GpuUsage.Set(gpuPercent);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                Console.Error.WriteLine($"[PerformanceSubsystem] {ex.Message}");
            }
        }
    }

    private static async Task<float> ReadRamUsageAsync()
    {
        var lines = await File.ReadAllLinesAsync("/proc/meminfo");
        long total = 0, available = 0;

        foreach (var line in lines)
        {
            if (line.StartsWith("MemTotal:"))
                total = ParseKb(line);
            else if (line.StartsWith("MemAvailable:"))
                available = ParseKb(line);
        }

        return total > 0 ? (float)(total - available) / total * 100f : 0f;
    }

    private static async Task<(long idle, long total)> ReadCpuStatsAsync()
    {
        var firstLine = (await File.ReadAllLinesAsync("/proc/stat"))[0];
        var parts = firstLine.Split(' ', StringSplitOptions.RemoveEmptyEntries);

        long idle = long.Parse(parts[4]);
        long total = parts.Skip(1).Sum(p => long.Parse(p));
        return (idle, total);
    }

    private static async Task<float> ReadJetsonGpuUsageAsync()
    {
        const string sysfsPath = "/sys/devices/gpu.0/load";

        if (File.Exists(sysfsPath))
        {
            var text = (await File.ReadAllTextAsync(sysfsPath)).Trim();
            if (int.TryParse(text, out int raw))
                return raw / 10f;
        }

        return await ReadGpuViaTegrastatsAsync();
    }

    private static async Task<float> ReadGpuViaTegrastatsAsync()
    {
        try
        {
            using var proc = new System.Diagnostics.Process
            {
                StartInfo = new System.Diagnostics.ProcessStartInfo
                {
                    FileName = "tegrastats",
                    Arguments = "--interval 1 --stop",
                    RedirectStandardOutput = true,
                    UseShellExecute = false,
                    CreateNoWindow = true
                }
            };
            proc.Start();
            string? line = await proc.StandardOutput.ReadLineAsync();
            await proc.WaitForExitAsync();

            if (line != null)
            {
                int idx = line.IndexOf("GR3D_FREQ", StringComparison.Ordinal);
                if (idx >= 0)
                {
                    var segment = line[(idx + 9)..].TrimStart();
                    var pctStr = new string(segment.TakeWhile(c => char.IsDigit(c) || c == '%').ToArray());
                    if (float.TryParse(pctStr.TrimEnd('%'), out float pct))
                        return pct;
                }
            }
        }
        catch { /* tegrastats not available */ }

        return 0f;
    }

    private static long ParseKb(string line)
    {
        var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        return parts.Length >= 2 && long.TryParse(parts[1], out long v) ? v : 0;
    }
}