namespace igvc_csharp.Subsystems.Arc.Streaming;

using System.Collections.Concurrent;
using System.Threading.Channels;

public static class JpegStreamRegistry
{
    // streamId -> (clientId -> channel)
    private static readonly ConcurrentDictionary<string, ConcurrentDictionary<Guid, Channel<byte[]>>> Streams = new();

    /// <summary>
    /// Registers a new per-client channel for the given stream and returns it.
    /// Each client gets its own channel so concurrent readers never race.
    /// </summary>
    public static (Guid clientId, Channel<byte[]> channel) Subscribe(string streamId)
    {
        var channel = Channel.CreateBounded<byte[]>(new BoundedChannelOptions(2)
        {
            FullMode    = BoundedChannelFullMode.DropOldest,
            SingleWriter = true,
            SingleReader = true
        });

        var clientId = Guid.NewGuid();
        var clients  = Streams.GetOrAdd(streamId, _ => new ConcurrentDictionary<Guid, Channel<byte[]>>());
        clients[clientId] = channel;

        Console.WriteLine($"[MJPEG] Client {clientId} subscribed to '{streamId}' ({clients.Count} total)");
        return (clientId, channel);
    }

    /// <summary>
    /// Removes the client channel and completes its writer so ReadAllAsync returns.
    /// </summary>
    public static void Unsubscribe(string streamId, Guid clientId)
    {
        if (!Streams.TryGetValue(streamId, out var clients)) return;

        if (clients.TryRemove(clientId, out var channel))
        {
            channel.Writer.TryComplete();
            Console.WriteLine($"[MJPEG] Client {clientId} unsubscribed from '{streamId}' ({clients.Count} remaining)");
        }
    }

    /// <summary>
    /// Pushes a JPEG frame to every active subscriber of the given stream.
    /// </summary>
    public static void Publish(string streamId, byte[] jpegBytes)
    {
        if (!Streams.TryGetValue(streamId, out var clients)) return;

        foreach (var (_, channel) in clients)
            channel.Writer.TryWrite(jpegBytes);
    }

    public static int SubscriberCount(string streamId) =>
        Streams.TryGetValue(streamId, out var clients) ? clients.Count : 0;
}