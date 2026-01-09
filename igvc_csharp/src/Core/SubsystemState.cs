namespace igvc_csharp.Core;

public enum SubsystemState : byte
{
    Initialized = 0,
    Ready = 1,
    Operating = 2,
    Errored = 3,
    Fatal = 4,
    Shutdown = 5,
    ShuttingDown = 6,
}
