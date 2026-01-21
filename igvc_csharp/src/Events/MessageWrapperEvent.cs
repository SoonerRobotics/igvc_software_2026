using igvc_csharp.Core;
using igvc_csharp.Utils.Messages;

namespace igvc_csharp.Events;

public record MessageWrapperEvent(MessageWrapper Wrapper) : IRobotEvent;