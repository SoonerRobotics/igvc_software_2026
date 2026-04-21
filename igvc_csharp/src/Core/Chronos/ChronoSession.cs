using System;
using System.Collections.Generic;
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

    // Video
    private readonly Dictionary<int, VideoWriter> _videoWriters = new();
    private readonly Dictionary<int, long> _videoFrameCounters = new();
    private readonly object _videoLock = new();

    public string RunId { get; }
    public string OutputDirectory { get; }
    public bool IsActive { get; private set; } = true;

    // ------------------------------------------------------------------
    internal ChronosSession(string outputDirectory, ushort sessionType, CancellationToken ct)
    {
        _cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        OutputDirectory = outputDirectory;
        Directory.CreateDirectory(outputDirectory);

        RunId = $"run_{DateTime.UtcNow:yyyyMMdd_HHmmss}_{sessionType}";
        string logPath = Path.Combine(outputDirectory, $"{RunId}.rlog");

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

    // Video
    internal void OpenCamera(int cameraId, int width, int height, double nominalFps)
    {
        lock (_videoLock)
        {
            if (_videoWriters.ContainsKey(cameraId))
                throw new InvalidOperationException($"Camera {cameraId} is already open.");

            string videoPath = Path.Combine(OutputDirectory, $"{RunId}_camera{cameraId}.avi");
            var writer = new VideoWriter(
                videoPath,
                VideoWriter.FourCC('X', 'V', 'I', 'D'),
                nominalFps,
                new Size(width, height)
            );

            if (!writer.IsOpened())
                throw new IOException($"Failed to open VideoWriter for camera {cameraId}.");

            _videoWriters[cameraId] = writer;
            _videoFrameCounters[cameraId] = 0;
        }
    }

    internal void WriteVideoFrame(int cameraId, Mat frame)
    {
        long tsNs = GetTimestampNs();
        lock (_videoLock)
        {
            if (!_videoWriters.TryGetValue(cameraId, out var writer))
                throw new InvalidOperationException($"Camera {cameraId} has not been opened.");

            writer.Write(frame);
            long frameIndex = _videoFrameCounters[cameraId]++;

            using var ms = new MemoryStream(12);
            using var bw = new BinaryWriter(ms);
            bw.Write(cameraId);
            bw.Write(frameIndex);
            EnqueueEntry(EntryTypeId.CameraFrameSync, tsNs, ms.ToArray());
        }
    }

    // Entry writing
    internal void EnqueueEntry(ushort typeId, byte[] payload)
        => EnqueueEntry(typeId, GetTimestampNs(), payload);

    internal void EnqueueEntry(ushort typeId, long tsNs, byte[] payload)
    {
        if (!IsActive) return;
        _channel.Writer.TryWrite((typeId, tsNs, payload));
    }

    // Emergency release (crash handler) — sync, best-effort
    internal void ReleaseVideoWriters()
    {
        lock (_videoLock)
        {
            foreach (var w in _videoWriters.Values)
                w.Release();
            _videoWriters.Clear();
        }
    }

    // Clean shutdown
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

    // Private helpers
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
