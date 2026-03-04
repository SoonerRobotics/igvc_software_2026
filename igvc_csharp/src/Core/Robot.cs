namespace igvc_csharp.Core;

using igvc_csharp.Core.Units;
using Microsoft.Extensions.Logging;

public sealed class Robot : BaseRobot
{
    private static readonly ILogger Logger = Logging.From<Robot>();

    /// <summary>
    /// A global instance of the robot.
    /// </summary>
    public static Robot Instance { get; internal set; } = null!;

    /// <summary>
    /// The current state of the robot.
    /// </summary>
    public RobotState State { get; } = new();
    
    // Other Information

    /// <summary>
    /// The current LatLng position of the robot. Null if GPS is lost.
    /// </summary>
    public LatLng? Location { get; set; } = null;
    
    /// <summary>
    /// The current heading of the robot (e.g. absolute compass direction it is facing). Null if GPS is lost.
    /// </summary>
    public Angle? Heading { get; set; } = null;

    public new void Dispose()
    {
        base.Dispose();
    }
    
    // Setters
    
    public void SetMobility(bool mobility)
    {
        State.MotionAllowed = mobility;
        CallSubsystemFunction(mobility ? "OnMobilityStart" : "OnMobilityStop");
    }

    public void SetEstopped(bool estopped)
    {
        State.Estopped = estopped;
        CallSubsystemFunction("OnRobotEstopped");
    }

    public void SetMode(RobotModeEnum mode)
    {
        var oldMode = State.Mode;
        State.Mode = mode;
        CallSubsystemFunction("OnRobotModeChanged", oldMode, mode);
    }

    public void SetMission(MissionEnum mission)
    {
        var oldMission = State.Mission;
        State.Mission = mission;
        CallSubsystemFunction("OnRobotModeChanged", oldMission, mission);
    }
}
