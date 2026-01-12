using System.Collections.Concurrent;
using System.Threading.Channels;

namespace igvc_csharp.Core;

public sealed class EventBus
{
    public static EventBus Instance { get; } = new();

    private readonly ConcurrentDictionary<Type, ConcurrentBag<ISubscription>> _subscriptions = new();

    public ChannelReader<T> Subscribe<T>(
        int capacity = 64,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest)
        where T : IRobotEvent
    {
        var channel = Channel.CreateBounded<T>(new BoundedChannelOptions(capacity)
        {
            SingleReader = true,
            SingleWriter = false,
            FullMode = fullMode
        });

        var bag = _subscriptions.GetOrAdd(typeof(T), _ => new ConcurrentBag<ISubscription>());
        bag.Add(new Subscription<T>(channel));

        return channel.Reader;
    }

    public void Publish<T>(T evt) where T : IRobotEvent
    {
        if (!_subscriptions.TryGetValue(typeof(T), out var subs))
        {
            return;
        }

        foreach (var sub in subs)
        {
            ((Subscription<T>)sub).TryWrite(evt);
        }
    }

    private interface ISubscription
    {
    }

    private sealed class Subscription<T>(Channel<T> channel) : ISubscription
        where T : IRobotEvent
    {
        public void TryWrite(T evt)
        {
            channel.Writer.TryWrite(evt);
        }
    }
}