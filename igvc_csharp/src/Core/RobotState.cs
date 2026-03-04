namespace igvc_csharp.Core;

using System.Threading;

public sealed class RobotState
{
    private int _mode;
    private int _motionAllowed;
    private int _mission;
    private int _isSimulation;
    private int _estopped;

    public RobotModeEnum Mode
    {
        get => (RobotModeEnum)Volatile.Read(ref _mode);
        set => Volatile.Write(ref _mode, (int)value);
    }

    public bool MotionAllowed
    {
        get => Volatile.Read(ref _motionAllowed) != 0;
        set => Volatile.Write(ref _motionAllowed, value ? 1 : 0);
    }

    public MissionEnum Mission
    {
        get => (MissionEnum)Volatile.Read(ref _mission);
        set => Volatile.Write(ref _mission, (int)value);
    }

    public bool IsSimulation
    {
        get => Volatile.Read(ref _isSimulation) != 0;
        set => Volatile.Write(ref _isSimulation, value ? 1 : 0);
    }

    public bool Estopped
    {
        get => Volatile.Read(ref _estopped) != 0;
        set => Volatile.Write(ref _estopped, value ? 1 : 0);
    }
}
