namespace igvc_csharp.External;

using System;
using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;
using System.Threading;

public sealed class SharedMemoryChannel : IDisposable
{
    private readonly MemoryMappedFile _mmf;
    private readonly MemoryMappedViewAccessor _accessor;
    private readonly Semaphore _semaphore;

    public SharedMemoryChannel(string name)
    {
        _mmf = MemoryMappedFile.CreateFromFile(
            $"/dev/shm/{name}",
            System.IO.FileMode.Open,
            null);

        _accessor = _mmf.CreateViewAccessor();
        _semaphore = new Semaphore(1, 1, name + "_sem");
    }

    public FrameHeader ReadHeader()
    {
        _semaphore.WaitOne();
        _accessor.Read(0, out FrameHeader header);
        _semaphore.Release();
        return header;
    }

    public void ReadBytes(long offset, byte[] buffer)
    {
        _semaphore.WaitOne();
        _accessor.ReadArray(offset, buffer, 0, buffer.Length);
        _semaphore.Release();
    }

    public void Dispose()
    {
        _accessor.Dispose();
        _mmf.Dispose();
        _semaphore.Dispose();
    }
}