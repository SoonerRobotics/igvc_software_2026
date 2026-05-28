
namespace igvc_csharp.src.subsystems.selfdrive;

public class SelfdriveMachine
{
    public enum SelfdriveGoal
    {
        TurnRight,
        TurnLeft,
        LaneKeep,
        ChangeLane,
        StopAtObstacle,
        TimedStopAtObstacle,
        StopAtObstacleUntilClear,
        LaneChangeAtObstacle,
    }

    public enum SelfdriveObstacles
    {
        Barrel,
        Stopsign,
        Tire,
        Pedestrian,
        WhiteLine
    }

    public enum SelfdriveLane
    {
        Left,
        Right
    }
}