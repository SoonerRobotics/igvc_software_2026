using System.Runtime.InteropServices;
using Google.FlatBuffers;
using igvc_csharp.Events;

namespace igvc_csharp.Utils.Messages;

public class MessageWrapper
{
    public MessageType Type { get; private set; }
    public byte[]? Data { get; private set; }

    private ByteBuffer? _buffer;
    
    private ByteBuffer GetByteBuffer()
    {
        if (!MemoryMarshal.TryGetArray(Data, out ArraySegment<byte> seg))
        {
            throw new InvalidOperationException("Non-array backed memory");
        }

        return new ByteBuffer(seg.Array!, seg.Offset);
    }

    public T As<T>() where T : struct
    {
        _buffer ??= GetByteBuffer();
        return FlatBufferRegistry.Resolve<T>(_buffer);
    }

    public MessageWrapperEvent Event()
    {
        return new MessageWrapperEvent(this);
    }
    
    public static MessageWrapper From(MessageType type, byte[] data)
    {
        return new MessageWrapper()
        {
            Type = type,
            Data = data
        };
    }
}