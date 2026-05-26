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

    public enum SelfdriveLane
    {
        Left,
        Right
    }
}