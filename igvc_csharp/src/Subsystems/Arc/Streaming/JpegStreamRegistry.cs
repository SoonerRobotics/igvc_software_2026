namespace igvc_csharp.Subsystems.Arc.Streaming;

using System.Collections.Concurrent;
using System.Threading.Channels;
using igvc_csharp.Core;

public static class JpegStreamRegistry
{
    private static readonly ConcurrentDictionary<string, Channel<byte[]>> Streams = new();

    public static Channel<byte[]> GetOrCreate(string streamId) =>
        Streams.GetOrAdd(streamId, _ => Channel.CreateBounded<byte[]>(
            new BoundedChannelOptions(1)          // only 1 frame buffered
            {
                FullMode = BoundedChannelFullMode.DropOldest,
                SingleWriter = false,
                SingleReader = false
            }));

    public static void Publish(string streamId, byte[] jpegBytes)
    {
        var channel = GetOrCreate(streamId);
        channel.Writer.TryWrite(jpegBytes);
    }
}