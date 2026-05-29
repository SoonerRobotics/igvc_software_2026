
namespace igvc_csharp.src.subsystems.selfdrive;

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
    None,
    Barrel,
    Stopsign,
    Tire,
    Pedestrian,
    WhiteLine,
    Any
}

public enum SelfdriveLane
{
    Left,
    Right
}

public enum QualificationTest
{
    LaneKeeping_Q1,
    LineDetection_Q2,
    LeftTurn_Q3,
    RightTurn_Q4
}

public enum SelfdriveTest
{
    PedestrianDetection_FI_1,
    TireDetection_FI_2,
    StopSignDetection_FII_1,
    LaneKeeping_FIII_1,
    LeftTurn_FIII_2,
    RightTurn_FIII_3,
    PullOut_FIV_1,
    PullIn_FIV_2,
    ParallelPark_FIV_3,
    StaticPedestrian_FV_1,
    DynamicPedestrian_FV_2,
    AvoidPedestrian_FV_3,
    BarrelLaneChange_FV_4,
    CurvedLaneKeeping_FVI_1,
    CurvedLaneChange_FVI_2,
    PotholeDetection_FVII_1
}