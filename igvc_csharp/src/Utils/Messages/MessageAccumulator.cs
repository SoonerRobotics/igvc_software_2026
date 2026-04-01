using System.Buffers.Binary;
using System.IO.Hashing;
using igvc_csharp.Core;

namespace igvc_csharp.Utils.Messages;

/// <summary>
/// Accumulates raw bytes and extracts framed messages.
///
/// Format:
/// [ ushort type ][ int length ][ payload ][ uint crc32c ]
///  
/// CRC covers the (type + length + payload)
/// </summary>
public sealed class MessageAccumulator(
    Endianness endianness,
    Action<MessageWrapper> onMessage,
    int initialCapacity = 4096,
    int maxMessageLength = 512 * 1024, // 512 KB
    int maxBufferSize = 1024 * 1024 // 1024 KB
)
{
    private readonly MemoryStream _stream = new(initialCapacity);

    private const int MagicSize = 4;
    private const int HeaderSize = MagicSize + 6;
    private const int CrcSize = 4;
    
    public void Append(ReadOnlySpan<byte> bytes)
    {
        if (_stream.Length + bytes.Length > maxBufferSize)
        {
            throw new InvalidOperationException("MessageAccumulator buffer overflow");
        }
        
        _stream.Write(bytes);
        Process();
    }

    private void Process()
    {
        _stream.Position = 0;

        while (_stream.Length - _stream.Position >= HeaderSize)
        {
            var buffer = _stream.GetBuffer();
            var span = buffer.AsSpan((int)_stream.Position);

            if (!span[..MagicSize].SequenceEqual(Configuration.NetworkingMagic))
            {
                // skip until magic
                _stream.Position += 1;
                continue;
            }
            
            var type = endianness == Endianness.Little
                ? BinaryPrimitives.ReadUInt16LittleEndian(span[MagicSize..])
                : BinaryPrimitives.ReadUInt16BigEndian(span[MagicSize..]);
            var length = endianness == Endianness.Little
                ? BinaryPrimitives.ReadInt32LittleEndian(span[(MagicSize + 2)..])
                : BinaryPrimitives.ReadInt32BigEndian(span[(MagicSize + 2)..]);

            if (length < 0 || length >= maxMessageLength)
            {
                throw new InvalidOperationException($"Invalid message length {length}");
            }

            var frameSize = HeaderSize + length + CrcSize;
            if (_stream.Length - _stream.Position < frameSize)
            {
                // Wait for more data;
                break;
            }

            var payloadSpan = span.Slice(HeaderSize, length);
            var receivedCrc = BinaryPrimitives.ReadUInt32LittleEndian(span.Slice(HeaderSize + length, CrcSize));
            Span<byte> crcData = stackalloc byte[HeaderSize + length];
            span[..HeaderSize].CopyTo(crcData);
            payloadSpan.CopyTo(crcData[HeaderSize..]);
            var computedCrc = Crc32.HashToUInt32(crcData);
            if (computedCrc != receivedCrc)
            {
                throw new InvalidOperationException("CRC mismatch in message");
            }

            var payload = new byte[length];
            payloadSpan.CopyTo(payload);

            onMessage(MessageWrapper.From((MessageType)type, payload));
            
            _stream.Position += frameSize;
        }

        // Compact remaining bytes
        if (_stream.Position > 0)
        {
            var remaining = _stream.Length - _stream.Position;

            if (remaining > 0)
            {
                Array.Copy(
                    _stream.GetBuffer(),
                    _stream.Position,
                    _stream.GetBuffer(),
                    0,
                    remaining);
            }

            _stream.SetLength(remaining);
            _stream.Position = 0;
        }
    }
}