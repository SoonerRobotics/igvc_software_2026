using System.Runtime.InteropServices;

namespace igvc_csharp.Core.Hardware;

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct CameraFrameHeader
{
    public uint SequenceNum;
    public long TimestampUs;
    public uint Width;
    public uint Height;
    public byte Valid;
}

/// <summary>
/// Reads BGR frames written by igvc_camera into POSIX shared memory.
/// Layout: [CameraFrameHeader | pixel bytes (BGR, 640×480)]
/// </summary>
public sealed class CameraFrameSharedMemoryReader : IDisposable
{
    public const int FrameWidth  = 640;
    public const int FrameHeight = 480;
    public const int FrameBytes  = FrameWidth * FrameHeight * 3; // BGR

    private static readonly int HeaderSize = Marshal.SizeOf<CameraFrameHeader>();
    private static readonly int TotalSize  = HeaderSize + FrameBytes;

    private readonly string _shmName;
    private readonly string _semName;

    private IntPtr _shmPtr = IntPtr.Zero;
    private IntPtr _semPtr = IntPtr.Zero;

    public bool IsOpen => _shmPtr != IntPtr.Zero && _semPtr != IntPtr.Zero;

    public CameraFrameSharedMemoryReader(string shmName, string semName)
    {
        _shmName = shmName;
        _semName = semName;
    }

    public bool TryOpen()
    {
        int fd = shm_open(_shmName, 0 /*O_RDONLY*/, 0);
        if (fd < 0) return false;

        _shmPtr = mmap(IntPtr.Zero, (nuint)TotalSize, 1 /*PROT_READ*/, 1 /*MAP_SHARED*/, fd, 0);
        close(fd);

        if (_shmPtr == MAP_FAILED)
        {
            _shmPtr = IntPtr.Zero;
            return false;
        }

        _semPtr = sem_open(_semName, 0, 0, 0);
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
    /// Returns null if the semaphore times out or there is no new frame since <paramref name="lastSeq"/>.
    /// The returned byte[] is a fresh copy of the BGR pixel data.
    /// </summary>
    public (CameraFrameHeader Header, byte[] Pixels)? TryRead(uint lastSeq, int timeoutMs = 100)
    {
        if (!IsOpen) return null;
        if (!SemTimedWait(_semPtr, timeoutMs)) return null;

        try
        {
            var header = Marshal.PtrToStructure<CameraFrameHeader>(_shmPtr);
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
    [DllImport("libc.so.6")]  private static extern int close(int fd);
    [DllImport("libc.so.6")]  private static extern IntPtr mmap(IntPtr addr, nuint length, int prot, int flags, int fd, long offset);
    [DllImport("libc.so.6")]  private static extern int munmap(IntPtr addr, nuint length);
    [DllImport("librt.so.1")] private static extern IntPtr sem_open(string name, int oflag, uint mode, uint value);
    [DllImport("librt.so.1")] private static extern int sem_close(IntPtr sem);
    [DllImport("librt.so.1")] private static extern int sem_post(IntPtr sem);

    [DllImport("librt.so.1")]
    private static extern int sem_timedwait(IntPtr sem, ref CameraTimespec absTimeout);

    private static bool SemTimedWait(IntPtr sem, int timeoutMs)
    {
        var ts = CameraTimespec.FromNow(timeoutMs);
        return sem_timedwait(sem, ref ts) == 0;
    }
}

[StructLayout(LayoutKind.Sequential)]
internal struct CameraTimespec
{
    public long TvSec;
    public long TvNsec;

    public static CameraTimespec FromNow(int offsetMs)
    {
        var now    = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        var target = now + offsetMs;
        return new CameraTimespec
        {
            TvSec  = target / 1000,
            TvNsec = (target % 1000) * 1_000_000L
        };
    }
}