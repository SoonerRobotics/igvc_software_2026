namespace igvc_csharp.Core;

public enum SubsystemState : byte
{
    /// <summary>
    /// Default SubsystemState state
    /// </summary>
    Initialized = 0,

    /// <summary>
    /// The subsystem is waiting for something to start, but is not actively trying to perform actions
    /// </summary>
    Idle = 1,

    /// <summary>
    /// The subsystem is actively trying to perform actions to move into a ready or operating state
    /// </summary>
    Starting = 2,

    /// <summary>
    /// The subsystem is ready to be operating but is waiting for a signal or something to happen
    /// </summary>
    Ready = 3,

    /// <summary>
    /// The subsystem is operating and actively performing operations
    /// </summary>
    Operating = 4,

    /// <summary>
    /// The subsystem has errored but is recoverable via user interaction or a signal
    /// </summary>
    Errored = 5,

    /// <summary>
    /// The subsystem is in a fatal state is not recoverable, the app/robot must be restarted
    /// </summary>
    Fatal = 6,
    
    /// <summary>
    /// The robot is starting the process of shutting down
    /// </summary>
    ShuttingDown = 7,

    /// <summary>
    /// The robot has fully shut down
    /// </summary>
    Shutdown = 8,
}
