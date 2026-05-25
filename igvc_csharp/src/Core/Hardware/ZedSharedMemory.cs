using System.Runtime.InteropServices;

namespace igvc_csharp.Core.Hardware;

// ─── Shared memory structs (must match C++ #pragma pack(push,1) layout) ───────

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct ZedFrameHeader
{
    public uint SequenceNum;
    public long TimestampUs;
    public uint Width;
    public uint Height;
    public byte Valid;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct ZedDepthRequest
{
    public uint RequestId;
    public int PixelX;
    public int PixelY;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct ZedDepthResponse
{
    public uint RequestId;
    public float X;
    public float Y;
    public float Z;
    public float Distance;
    public byte Valid;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct ZedDepthChannel
{
    public ZedDepthRequest Request;
    public ZedDepthResponse Response;
}

// ─── Frame reader ─────────────────────────────────────────────────────────────

/// <summary>
/// Reads RGB (BGRA) frames written by igvc_zed into POSIX shared memory.
/// The SHM region layout is:  [ZedFrameHeader | pixel bytes (BGRA, 1280×720)]
/// </summary>
public sealed class ZedFrameSharedMemoryReader : IDisposable
{
    private const string ShmName = "/zed_frame";
    private const string SemName = "/zed_frame_sem";

    public const int FrameWidth = 1280;
    public const int FrameHeight = 720;
    public const int FrameBytes = FrameWidth * FrameHeight * 4; // BGRA

    private static readonly int HeaderSize = Marshal.SizeOf<ZedFrameHeader>();
    private static readonly int TotalSize = HeaderSize + FrameBytes;

    private IntPtr _shmPtr = IntPtr.Zero;
    private IntPtr _semPtr = IntPtr.Zero;

    public bool IsOpen => _shmPtr != IntPtr.Zero && _semPtr != IntPtr.Zero;

    public bool TryOpen()
    {
        // O_RDONLY = 0, O_RDWR = 2
        int fd = shm_open(ShmName, 0 /*O_RDONLY*/, 0);
        if (fd < 0) return false;

        _shmPtr = mmap(IntPtr.Zero, (nuint)TotalSize, 1 /*PROT_READ*/, 1 /*MAP_SHARED*/, fd, 0);
        close(fd);

        if (_shmPtr == MAP_FAILED)
        {
            _shmPtr = IntPtr.Zero;
            return false;
        }

        _semPtr = sem_open(SemName, 0 /*O_RDONLY flags, existing sem*/, 0, 0);
        if (_semPtr == SEM_FAILED)
        {
            munmap(_shmPtr, (nuint)TotalSize);
            _shmPtr = IntPtr.Zero;
            _semPtr = IntPtr.Zero;
            return false;
        }

        return true;
    }

    /// <summary>
    /// Attempts to read a frame. Returns null if the sem times out or no new data.
    /// The returned byte[] is a fresh copy of the BGRA pixel data — safe to hand to OpenCvSharp.
    /// </summary>
    public (ZedFrameHeader Header, byte[] Pixels)? TryRead(uint lastSeq, int timeoutMs = 100)
    {
        if (!IsOpen) return null;

        if (!SemTimedWait(_semPtr, timeoutMs)) return null;

        try
        {
            var header = Marshal.PtrToStructure<ZedFrameHeader>(_shmPtr);
            if (header.Valid == 0 || header.SequenceNum == lastSeq) return null;

            var pixels = new byte[FrameBytes];
            Marshal.Copy(_shmPtr + HeaderSize, pixels, 0, FrameBytes);

            return (header, pixels);
        }
        finally
        {
            sem_post(_semPtr);
        }
    }

    public void Close()
    {
        if (_shmPtr != IntPtr.Zero) { munmap(_shmPtr, (nuint)TotalSize); _shmPtr = IntPtr.Zero; }
        if (_semPtr != IntPtr.Zero && _semPtr != SEM_FAILED) { sem_close(_semPtr); _semPtr = IntPtr.Zero; }
    }

    public void Dispose() => Close();

    // ── POSIX P/Invoke ────────────────────────────────────────────────────────
    private static readonly IntPtr MAP_FAILED = new(-1);
    private static readonly IntPtr SEM_FAILED = new(-1);

    [DllImport("librt.so.1")] private static extern int shm_open(string name, int oflag, uint mode);
    [DllImport("libc.so.6")] private static extern int close(int fd);
    [DllImport("libc.so.6")] private static extern IntPtr mmap(IntPtr addr, nuint length, int prot, int flags, int fd, long offset);
    [DllImport("libc.so.6")] private static extern int munmap(IntPtr addr, nuint length);
    [DllImport("librt.so.1")] private static extern IntPtr sem_open(string name, int oflag, uint mode, uint value);
    [DllImport("librt.so.1")] private static extern int sem_close(IntPtr sem);
    [DllImport("librt.so.1")] private static extern int sem_post(IntPtr sem);

    [DllImport("librt.so.1")]
    private static extern int sem_timedwait(IntPtr sem, ref Timespec absTimeout);

    private static bool SemTimedWait(IntPtr sem, int timeoutMs)
    {
        var ts = Timespec.FromNow(timeoutMs);
        return sem_timedwait(sem, ref ts) == 0;
    }
}

// ─── Depth channel reader/writer ──────────────────────────────────────────────

/// <summary>
/// Writes depth pixel requests and reads back 3-D point responses from
/// the shared memory channel maintained by igvc_zed.
/// </summary>
public sealed class ZedDepthSharedMemoryChannel : IDisposable
{
    private const string ShmName = "/zed_depth";
    private const string SemName = "/zed_depth_sem";

    private static readonly int ChannelSize = Marshal.SizeOf<ZedDepthChannel>();

    private IntPtr _shmPtr = IntPtr.Zero;
    private IntPtr _semPtr = IntPtr.Zero;

    public bool IsOpen => _shmPtr != IntPtr.Zero && _semPtr != IntPtr.Zero;

    public bool TryOpen()
    {
        int fd = shm_open(ShmName, 2 /*O_RDWR*/, 0);
        if (fd < 0) return false;

        // PROT_READ|PROT_WRITE = 3
        _shmPtr = mmap(IntPtr.Zero, (nuint)ChannelSize, 3, 1 /*MAP_SHARED*/, fd, 0);
        close(fd);

        if (_shmPtr == MAP_FAILED)
        {
            _shmPtr = IntPtr.Zero;
            return false;
        }

        _semPtr = sem_open(SemName, 0, 0, 0);
        if (_semPtr == SEM_FAILED)
        {
            munmap(_shmPtr, (nuint)ChannelSize);
            _shmPtr = IntPtr.Zero;
            _semPtr = IntPtr.Zero;
            return false;
        }

        return true;
    }

    /// <summary>
    /// Writes a depth request for <paramref name="pixelX"/>, <paramref name="pixelY"/>
    /// and waits up to <paramref name="timeoutMs"/> for C++ to respond.
    /// Returns null if the request times out or the point is invalid.
    /// </summary>
    public ZedDepthResponse? RequestDepth(uint requestId, int pixelX, int pixelY, int timeoutMs = 500)
    {
        if (!IsOpen) return null;

        // Write the request under the sem
        if (!SemTimedWait(_semPtr, 200)) return null;

        var channel = Marshal.PtrToStructure<ZedDepthChannel>(_shmPtr);
        channel.Request.RequestId = requestId;
        channel.Request.PixelX = pixelX;
        channel.Request.PixelY = pixelY;
        Marshal.StructureToPtr(channel, _shmPtr, false);

        sem_post(_semPtr);

        // Poll for the matching response
        var deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);
        while (DateTime.UtcNow < deadline)
        {
            Thread.Sleep(5);

            if (!SemTimedWait(_semPtr, 50)) continue;

            var resp = Marshal.PtrToStructure<ZedDepthResponse>(_shmPtr + Marshal.OffsetOf<ZedDepthChannel>(nameof(ZedDepthChannel.Response)).ToInt32());
            sem_post(_semPtr);

            if (resp.RequestId == requestId)
                return resp.Valid == 1 ? resp : null;
        }

        return null; // timed out
    }

    public void Close()
    {
        if (_shmPtr != IntPtr.Zero) { munmap(_shmPtr, (nuint)ChannelSize); _shmPtr = IntPtr.Zero; }
        if (_semPtr != IntPtr.Zero && _semPtr != SEM_FAILED) { sem_close(_semPtr); _semPtr = IntPtr.Zero; }
    }

    public void Dispose() => Close();

    // ── POSIX P/Invoke (same set as above — keep in sync if you share a file) ─
    private static readonly IntPtr MAP_FAILED = new(-1);
    private static readonly IntPtr SEM_FAILED = new(-1);

    [DllImport("librt.so.1")] private static extern int shm_open(string name, int oflag, uint mode);
    [DllImport("libc.so.6")] private static extern int close(int fd);
    [DllImport("libc.so.6")] private static extern IntPtr mmap(IntPtr addr, nuint length, int prot, int flags, int fd, long offset);
    [DllImport("libc.so.6")] private static extern int munmap(IntPtr addr, nuint length);
    [DllImport("librt.so.1")] private static extern IntPtr sem_open(string name, int oflag, uint mode, uint value);
    [DllImport("librt.so.1")] private static extern int sem_close(IntPtr sem);
    [DllImport("librt.so.1")] private static extern int sem_post(IntPtr sem);

    [DllImport("librt.so.1")]
    private static extern int sem_timedwait(IntPtr sem, ref Timespec absTimeout);

    private static bool SemTimedWait(IntPtr sem, int timeoutMs)
    {
        var ts = Timespec.FromNow(timeoutMs);
        return sem_timedwait(sem, ref ts) == 0;
    }
}

// ─── Shared helper ────────────────────────────────────────────────────────────

[StructLayout(LayoutKind.Sequential)]
internal struct Timespec
{
    public long TvSec;
    public long TvNsec;

    public static Timespec FromNow(int offsetMs)
    {
        var now = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        var target = now + offsetMs;
        return new Timespec
        {
            TvSec = target / 1000,
            TvNsec = (target % 1000) * 1_000_000L
        };
    }
}