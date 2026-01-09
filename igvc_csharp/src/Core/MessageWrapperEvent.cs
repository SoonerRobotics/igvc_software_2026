using igvc_csharp.MessageUtils;

namespace igvc_csharp.Core;

public record MessageWrapperEvent(MessageWrapper Wrapper) : IRobotEvent;