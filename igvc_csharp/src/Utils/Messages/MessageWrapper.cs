using System;
using System.Buffers;
using System.Runtime.InteropServices;
using Google.FlatBuffers;
using igvc_csharp.Events;

namespace igvc_csharp.Utils.Messages;

public sealed class MessageWrapper : IDisposable
{
    public MessageType Type { get; private set; }

    public byte[]? Data { get; private set; }

    public int Length { get; private set; }

    private bool _pooled;
    private int _disposed;
    private ByteBuffer? _buffer;

    private ByteBuffer GetByteBuffer()
    {
        if (!MemoryMarshal.TryGetArray<byte>(Data, out var seg))
            throw new InvalidOperationException("Non-array backed memory");
        return new ByteBuffer(seg.Array!, seg.Offset);
    }

    public T As<T>() where T : struct
    {
        _buffer ??= GetByteBuffer();
        return FlatBufferRegistry.Resolve<T>(_buffer);
    }

    public MessageWrapperEvent Event() => new(this);

    public static MessageWrapper From(MessageType type, byte[] data)
    {
        return new MessageWrapper
        {
            Type = type,
            Data = data,
            Length = data.Length,
            _pooled = false
        };
    }

    public static MessageWrapper From(MessageType type, ByteBuffer data)
        => From(type, data.ToFullArray());

    public static MessageWrapper FromPooled(MessageType type, byte[] rentedArray, int length)
    {
        return new MessageWrapper
        {
            Type = type,
            Data = rentedArray,
            Length = length,
            _pooled = true
        };
    }

    public void Dispose()
    {
        if (System.Threading.Interlocked.Exchange(ref _disposed, 1) == 1) return;
        if (_pooled && Data != null)
            ArrayPool<byte>.Shared.Return(Data);
    }
}