using igvc_csharp.Core;

namespace igvc_csharp.Events;

public sealed record ConfigChangedEvent(
    string Path,
    object Value
) : IRobotEvent;