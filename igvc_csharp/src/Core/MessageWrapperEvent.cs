using igvc_csharp.Messages;

namespace igvc_csharp.Core;

public record MessageWrapperEvent(MessageWrapper Wrapper) : IRobotEvent;