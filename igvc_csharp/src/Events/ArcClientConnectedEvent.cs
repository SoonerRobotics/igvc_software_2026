using igvc_csharp.Core;

namespace igvc_csharp.Events;

/// <summary>
/// Published by ArcSubsystem immediately after a WebSocket client successfully
/// connects and is registered. Subscribers (e.g. ArcConfigHandler) can use
/// this to push an initial state snapshot to the new client.
/// </summary>
public sealed class ArcClientConnectedEvent : IRobotEvent
{
    public Guid ClientId { get; }

    public ArcClientConnectedEvent(Guid clientId)
    {
        ClientId = clientId;
    }
}