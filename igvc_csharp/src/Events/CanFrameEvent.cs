using igvc_csharp.Core;
using SocketCANSharp;

namespace igvc_csharp.Events;

public class CanFrameEvent(CanFrame frame) : IRobotEvent;
