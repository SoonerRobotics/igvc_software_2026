using igvc_csharp.Core;
using igvc_csharp.MessageUtils;

namespace igvc_csharp.Events;

public record MessageWrapperEvent(MessageWrapper Wrapper) : IRobotEvent;