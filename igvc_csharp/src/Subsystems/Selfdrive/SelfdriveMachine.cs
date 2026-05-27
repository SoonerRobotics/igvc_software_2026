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

    public enum SelfdriveState
    {
        LaneKeeping,
        Stopping,
        Turning
    }

    public enum SelfdriveLane
    {
        Left,
        Right
    }
}