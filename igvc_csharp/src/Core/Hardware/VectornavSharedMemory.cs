namespace igvc_csharp.Core.Hardware;

using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential, Pack = 1)]
internal struct VectorNavReport
{
    // GPS
    public double Latitude;
    public double Longitude;
    public double Altitude;
    public float VelNorthMs;
    public float VelEastMs;
    public float VelDownMs;
    public byte NumSats;
    public byte GpsFix;

    // Attitude
    public float Yaw;
    public float Pitch;
    public float Roll;

    // Metadata
    public long TimestampUs;
    public uint SequenceNum;
    public byte Valid;
}

internal sealed class VectorNavSharedMemoryReader : IDisposable
{
    private const string ShmName = "/vectornav_report";
    private const string SemName = "/vectornav_sem";
    private static readonly IntPtr SemFailed = new(-1);
    private static readonly int ShmSize = Marshal.SizeOf<VectorNavReport>();

    private const int O_RDONLY = 0;
    private const int PROT_READ = 1;
    private const int MAP_SHARED = 1;
    private const int CLOCK_REALTIME = 0;

    private IntPtr _map = IntPtr.Zero;
    private IntPtr _sem = new(-1);
    private bool _disposed;

    [DllImport("librt.so.1")]
    static extern int shm_open(string name, int oflag, uint mode);

    [DllImport("libc.so.6")]
    static extern IntPtr mmap(IntPtr addr, int length, int prot, int flags, int fd, int offset);

    [DllImport("libc.so.6")]
    static extern int munmap(IntPtr addr, int length);

    [DllImport("libc.so.6")]
    static extern int close(int fd);

    [DllImport("librt.so.1")]
    static extern IntPtr sem_open(string name, int oflag);

    [DllImport("librt.so.1")]
    static extern int sem_timedwait(IntPtr sem, ref Timespec ts);

    [DllImport("librt.so.1")]
    static extern int sem_post(IntPtr sem);

    [DllImport("librt.so.1")]
    static extern int sem_close(IntPtr sem);

    [DllImport("libc.so.6")]
    static extern int clock_gettime(int clockId, ref Timespec ts);

    [StructLayout(LayoutKind.Sequential)]
    private struct Timespec
    {
        public long TvSec;
        public long TvNsec;
    }

    /// <summary>Try to open the shared memory and semaphore. Returns false if not ready yet.</summary>
    public bool TryOpen()
    {
        var fd = shm_open(ShmName, O_RDONLY, 0);
        if (fd < 0) return false;

        var map = mmap(IntPtr.Zero, ShmSize, PROT_READ, MAP_SHARED, fd, 0);
        close(fd);

        if (map == new IntPtr(-1)) return false;

        var sem = sem_open(SemName, 0);
        if (sem == SemFailed)
        {
            munmap(map, ShmSize);
            return false;
        }

        _map = map;
        _sem = sem;
        return true;
    }

    public bool IsOpen => _map != IntPtr.Zero && _sem != SemFailed;

    /// <summary>
    /// Tries to read the next report from shared memory.
    /// Returns null if no new data is available within the timeout.
    /// </summary>
    public VectorNavReport? TryRead(int timeoutMs = 100)
    {
        if (!IsOpen) throw new InvalidOperationException("Shared memory is not open.");

        var ts = new Timespec();
        clock_gettime(CLOCK_REALTIME, ref ts);
        ts.TvNsec += timeoutMs * 1_000_000L;
        if (ts.TvNsec >= 1_000_000_000L)
        {
            ts.TvSec++;
            ts.TvNsec -= 1_000_000_000L;
        }

        if (sem_timedwait(_sem, ref ts) != 0)
            return null;

        var report = Marshal.PtrToStructure<VectorNavReport>(_map);
        sem_post(_sem);
        return report;
    }

    public void Close()
    {
        if (_sem != SemFailed)
        {
            sem_close(_sem);
            _sem = SemFailed;
        }

        if (_map == IntPtr.Zero) return;
        munmap(_map, ShmSize);
        _map = IntPtr.Zero;
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        if (_sem != SemFailed)
        {
            sem_close(_sem);
            _sem = SemFailed;
        }

        if (_map != IntPtr.Zero)
        {
            munmap(_map, ShmSize);
            _map = IntPtr.Zero;
        }
    }
}