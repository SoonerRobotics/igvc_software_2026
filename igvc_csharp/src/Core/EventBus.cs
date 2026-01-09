using System.Collections.Concurrent;
using System.Threading.Channels;

namespace igvc_csharp.Core;

public sealed class EventBus
{
    public static EventBus Instance { get; } = new ();
    
    private readonly ConcurrentDictionary<Type, object> _channels = new();

    public Channel<T> GetChannel<T>() where T: IRobotEvent
    {
        return (Channel<T>)_channels.GetOrAdd(
            typeof(T),
            _ => Channel.CreateBounded<T>(new BoundedChannelOptions(64)
            {
                SingleWriter = false,
                SingleReader = false,
                FullMode = BoundedChannelFullMode.DropOldest
            })
        );
    }

    public static bool Write<TChannel>(TChannel item) where TChannel : IRobotEvent
    {
        return Instance.GetChannel<TChannel>().Writer.TryWrite(item);
    }
}