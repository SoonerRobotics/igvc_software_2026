using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;
using OpenCvSharp;

namespace igvc_csharp.Core.Chronos;

internal sealed class ChronosSession : IAsyncDisposable
{
    private const int FlushIntervalMs = 500;
    private const int ChannelCapacity = 512;

    // Binary log
    private readonly FileStream _logStream;
    private readonly BinaryWriter _logWriter;

    // Async write pipeline
    private readonly Channel<(ushort type, long tsNs, byte[] payload)> _channel;
    private readonly Task _writerTask;
    private readonly Task _flushTask;
    private readonly CancellationTokenSource _cts = new();

    // Timing
    private readonly System.Diagnostics.Stopwatch _stopwatch = System.Diagnostics.Stopwatch.StartNew();
    private readonly long _swOriginTicks;

    // Video — ffmpeg pipe backend (replaces OpenCvSharp VideoWriter)
    private readonly Dictionary<int, Process> _ffmpegProcesses = new();
    private readonly Dictionary<int, long> _videoFrameCounters = new();
    private readonly object _videoLock = new();

    public string RunId { get; }
    public string OutputDirectory { get; }
    public bool IsActive { get; private set; } = true;

    // ------------------------------------------------------------------
    internal ChronosSession(string outputDirectory, ushort sessionType, CancellationToken ct)
    {
        _cts = CancellationTokenSource.CreateLinkedTokenSource(ct);

        // Each session gets its own folder
        RunId = $"run_{DateTime.UtcNow:yyyyMMdd_HHmmss}_{sessionType}";
        OutputDirectory = Path.Combine(outputDirectory, RunId);
        Directory.CreateDirectory(OutputDirectory);
        string logPath = Path.Combine(OutputDirectory, "session.rlog");

        _logStream = new FileStream(
            logPath,
            FileMode.Create,
            FileAccess.Write,
            FileShare.Read,
            bufferSize: 65536
        );
        _logWriter = new BinaryWriter(_logStream);
        _swOriginTicks = _stopwatch.ElapsedTicks;

        WriteFileHeader();

        _channel = Channel.CreateBounded<(ushort, long, byte[])>(ChannelCapacity);
        _writerTask = RunWriterAsync(_cts.Token);
        _flushTask = RunPeriodicFlushAsync(_cts.Token);

        EnqueueEntry(EntryTypeId.SessionStart, Array.Empty<byte>());
    }

    // ------------------------------------------------------------------
    // Video — ffmpeg pipe backend
    // ------------------------------------------------------------------

    /// <summary>
    /// Opens an ffmpeg process for the given camera and pipes raw BGR24 frames to it.
    /// Tries the Jetson HW encoder (h264_nvmec) first; fall back to nvv4l2h264enc or
    /// libx264 if that encoder is unavailable on your build.
    /// </summary>
    internal void OpenCamera(int cameraId, int width, int height, double nominalFps, string pixFmt = "bgr24")
    {
        // lock (_videoLock)
        // {
        //     if (_ffmpegProcesses.ContainsKey(cameraId))
        //         throw new InvalidOperationException($"Camera {cameraId} is already open.");

        //     string videoPath = Path.Combine(OutputDirectory, $"camera{cameraId}.mp4");

        //     var psi = new ProcessStartInfo
        //     {
        //         FileName = "ffmpeg",
        //         Arguments = string.Join(" ",
        //             "-y",
        //             "-f rawvideo",
        //             $"-pix_fmt {pixFmt}",   // <-- use parameter
        //             $"-video_size {width}x{height}",
        //             $"-framerate {nominalFps}",
        //             "-i pipe:0",
        //             "-c:v libx264",
        //             "-preset ultrafast",
        //             "-crf 23",
        //             "-pix_fmt yuv420p",     // <-- add this for output
        //             "-an",
        //             videoPath
        //         ),
        //         UseShellExecute = false,
        //         RedirectStandardInput = true,
        //         RedirectStandardError = true,
        //     };

        //     var process = Process.Start(psi)
        //         ?? throw new IOException($"Failed to start ffmpeg for camera {cameraId}.");

        //     // Drain stderr on a background thread so the pipe never blocks
        //     process.ErrorDataReceived += (_, e) =>
        //     {
        //         if (e.Data != null)
        //             Console.Error.WriteLine($"[ffmpeg cam{cameraId}] {e.Data}");
        //     };
        //     process.BeginErrorReadLine();

        //     _ffmpegProcesses[cameraId] = process;
        //     _videoFrameCounters[cameraId] = 0;
        // }
    }

    /// <summary>
    /// Writes a single BGR24 Mat frame to the ffmpeg stdin pipe for the given camera.
    /// Requires AllowUnsafeBlocks in the project file.
    /// </summary>
    internal unsafe void WriteVideoFrame(int cameraId, Mat frame)
    {
        // lock (_videoLock)
        // {
        //     if (!_ffmpegProcesses.TryGetValue(cameraId, out var process) || process.HasExited)
        //         return;  // silently drop frame if ffmpeg died

        //     try
        //     {
        //         long byteCount = frame.Total() * frame.ElemSize();
        //         var span = new ReadOnlySpan<byte>((void*)frame.Data, (int)byteCount);
        //         process.StandardInput.BaseStream.Write(span);

        //         long frameIndex = _videoFrameCounters[cameraId]++;
        //         using var ms = new MemoryStream(12);
        //         using var bw = new BinaryWriter(ms);
        //         bw.Write(cameraId);
        //         bw.Write(frameIndex);
        //         EnqueueEntry(EntryTypeId.CameraFrameSync, GetTimestampNs(), ms.ToArray());
        //     }
        //     catch (IOException)
        //     {
        //         // ffmpeg died mid-session; remove it so we stop trying
        //         _ffmpegProcesses.Remove(cameraId);
        //     }
        // }
    }

    // ------------------------------------------------------------------
    // Entry writing
    // ------------------------------------------------------------------

    internal void EnqueueEntry(ushort typeId, byte[] payload)
        => EnqueueEntry(typeId, GetTimestampNs(), payload);

    internal void EnqueueEntry(ushort typeId, long tsNs, byte[] payload)
    {
        if (!IsActive) return;
        _channel.Writer.TryWrite((typeId, tsNs, payload));
    }

    // ------------------------------------------------------------------
    // Emergency release (crash handler) — sync, best-effort
    // ------------------------------------------------------------------

    internal void ReleaseVideoWriters()
    {
        lock (_videoLock)
        {
            foreach (var (id, process) in _ffmpegProcesses)
            {
                try
                {
                    // Closing stdin signals EOF; ffmpeg will finalize and mux the file
                    process.StandardInput.BaseStream.Close();
                    process.WaitForExit(5_000);
                }
                catch (Exception ex)
                {
                    Console.Error.WriteLine($"[ffmpeg cam{id}] shutdown error: {ex.Message}");
                    try { process.Kill(); } catch { /* ignore */ }
                }
                finally
                {
                    process.Dispose();
                }
            }
            _ffmpegProcesses.Clear();
        }
    }

    // ------------------------------------------------------------------
    // Clean shutdown
    // ------------------------------------------------------------------

    public async ValueTask DisposeAsync()
    {
        if (!IsActive) return;
        IsActive = false;

        EnqueueEntry(EntryTypeId.SessionEnd, Array.Empty<byte>());
        _channel.Writer.Complete();
        await _writerTask;

        _cts.Cancel();
        try { await _flushTask; } catch (OperationCanceledException) { }

        _logStream.Flush(flushToDisk: true);
        _logWriter.Dispose();
        _logStream.Dispose();

        ReleaseVideoWriters();
    }

    // ------------------------------------------------------------------
    // Private helpers
    // ------------------------------------------------------------------

    private void WriteFileHeader()
    {
        _logWriter.Write(0x524C4F47u); // Magic "RLOG"
        _logWriter.Write((ushort)0x0001); // Version
        _logWriter.Write((ushort)0);      // Reserved
        _logWriter.Write(DateTimeOffset.UtcNow.ToUnixTimeMilliseconds());
        _logWriter.Write(_swOriginTicks);
        _logWriter.Write((ulong)0); // Reserved
        _logWriter.Flush();
    }

    private long GetTimestampNs()
    {
        long ticks = _stopwatch.ElapsedTicks - _swOriginTicks;
        return (long)(ticks * (1_000_000_000.0 / System.Diagnostics.Stopwatch.Frequency));
    }

    private async Task RunWriterAsync(CancellationToken ct)
    {
        await foreach (var (type, tsNs, payload) in _channel.Reader.ReadAllAsync(ct))
        {
            try
            {
                _logWriter.Write(type);
                _logWriter.Write(tsNs);
                _logWriter.Write((uint)payload.Length);
                _logWriter.Write(payload);
            }
            catch (Exception ex)
            {
                Console.Error.WriteLine($"[RobotLogger] Write error: {ex.Message}");
            }
        }
    }

    private async Task RunPeriodicFlushAsync(CancellationToken ct)
    {
        try
        {
            while (!ct.IsCancellationRequested)
            {
                await Task.Delay(FlushIntervalMs, ct);
                try { _logStream.Flush(flushToDisk: true); }
                catch { /* ignore */ }
            }
        }
        catch (OperationCanceledException) { }
    }
}