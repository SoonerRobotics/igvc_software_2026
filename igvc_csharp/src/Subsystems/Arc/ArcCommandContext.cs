using Messages.Arc;

namespace igvc_csharp.Subsystems.Arc;

/// <summary>
/// Pairs an incoming ArcCommand with the WebSocket client that sent it.
/// Passed to [ArcCommand] handlers so they can send targeted replies
/// without needing a client ID in the FlatBuffer wire format.
/// </summary>
public readonly struct ArcCommandContext
{
    /// <summary>The WebSocket client that sent this command.</summary>
    public Guid ClientId { get; init; }

    /// <summary>The decoded ArcCommand FlatBuffer.</summary>
    public ArcCommand Command { get; init; }
}