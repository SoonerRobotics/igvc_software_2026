using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware;
using igvc_csharp.Core.Config;
using igvc_csharp.Subsystems.Navigation;
using igvc_csharp.src.Subsystems.selfdrive;
using igvc_csharp.src.selfdrive.actions;
using igvc_csharp.src.Subsystems.selfdrive.actions;
using igvc_csharp.Core.Units;
using Messages;
using igvc_csharp.Utils;
using Microsoft.Extensions.Logging;

using Mat = OpenCvSharp.Mat;

namespace igvc_csharp.src.subsystems.selfdrive;

[Subsystem("SelfdriveSubsystem", Disabled = true)]
public class SelfdriveSubsystem(CanbusSubsystem? canbus) : SubsystemBase
{
    // --- Config ---

    [Config("selfdrive.forward_speed")]
    public static LinearVelocity ForwardSpeed = LinearVelocityUnit.MilesPerHour.Of(1);

    [Config("selfdrive.lane_change_speed")]
    public LinearVelocity LaneChangeSpeed = LinearVelocityUnit.MilesPerHour.Of(0.5);

    [Config("selfdrive.turn_speed")]
    public static AngularVelocity TurnSpeed = AngularVelocityUnit.DegreesPerSecond.Of(30);

    [Config("selfdrive.feet_to_stop")]
    public Distance ObstacleStopDistance = DistanceUnit.Feet.Of(2);

    [Config("selfdrive.pull_out_direction")]
    public int PullOutDirection = 0; // positive = left

    [Config("selfdrive.obstacle_reaction_distance")]
    public Distance ObstacleReactionDistance = DistanceUnit.Feet.Of(12);

    [Config("selfdrive.lane_change_timeout")]
    public ulong LaneChangeTimeoutMs = 3000;

    [Config("selfdrive.qualification_mode")]
    public bool QualificationMode = true;

    [Config("selfdrive.qualification_test")]
    public QualificationTest QualTest = QualificationTest.LaneKeeping_Q1;

    // --- State ---

    public SubsystemProperty<SelfdriveGoal> mGoal = new("goal", SelfdriveGoal.LaneKeep);
    public SubsystemProperty<SelfdriveLane> mLane = new("lane", SelfdriveLane.Right);
    public SubsystemProperty<SelfdriveTest> mTest = new("functional_test", SelfdriveTest.PedestrianDetection_FI_1);

    private bool shouldBuild = true;
    private readonly SelfdriveContext _context = new()
    {
        Canbus = canbus ?? throw new ArgumentNullException(nameof(canbus),
            "SelfdriveSubsystem requires a CanbusSubsystem."),
        YoloDetections = new(),
    };

    // Protects all _context.Last*Frame writes and YoloDetections mutations.
    private readonly object _contextLock = new();

    // The currently running action; null means we need to build a new one.
    private ISelfdriveAction? _currentAction;

    // --- Lifecycle ---

    public override Task Init(CancellationToken token)
    {
        Subscribe<YoloDetectionEvent>(OnDetectionEvent, token);
        SubscribeImage("zed", OnZedImageReceived, token);
        SubscribeImage("center", OnCenterImageReceived, token);
        SubscribeImage("combined_filtered", OnFilteredImageReceived, token);

        _ = Task.Run(() => SelfdriveLoop(token), token);
        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        if (old.Mode != RobotModeEnum.Autonomous && updated.Mode == RobotModeEnum.Autonomous)
        {
            Logger.LogInformation("Entered Autonomous mode, resetting selfdrive action.");
            shouldBuild = true;
        }

        return Task.CompletedTask;
    }

    public override void OnPositionChanged(RobotPosition position) { }

    // --- Image subscriptions ---

    public Task OnZedImageReceived(ImageFrame frame, CancellationToken ct)
    {
        SwapFrame(ref _context.LastZedFrame, CvUtils.AsMat(frame));
        return Task.CompletedTask;
    }

    public Task OnCenterImageReceived(ImageFrame frame, CancellationToken ct)
    {
        SwapFrame(ref _context.LastCenterFrame, CvUtils.AsMat(frame));
        return Task.CompletedTask;
    }

    public Task OnFilteredImageReceived(ImageFrame frame, CancellationToken ct)
    {
        SwapFrame(ref _context.LastFilteredFrame, CvUtils.AsMat(frame));
        return Task.CompletedTask;
    }

    // Thread-safe Mat swap — disposes the old frame under the lock.
    private void SwapFrame(ref Mat? slot, Mat incoming)
    {
        Mat? old;
        lock (_contextLock)
        {
            old = slot;
            slot = incoming;
        }
        old?.Dispose();
    }

    // --- Detection events ---

    public Task OnDetectionEvent(YoloDetectionEvent e, CancellationToken token)
    {
        lock (_contextLock)
        {
            _context.YoloDetections[e.label] = e;       // AddOrReplace, no manual Remove needed
            _context.DetectionTracker.Feed(e);
        }
        return Task.CompletedTask;
    }

    // --- Main loop ---

    private async Task SelfdriveLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            try
            {
                if (!IsReadyToDrive())
                {
                    Logger.LogInformation("Waiting...");
                    ResetAction();
                    await Task.Delay(5_000, token);
                    continue;
                }

                if (shouldBuild)
                {
                    EnsureActionBuilt();
                    shouldBuild = false;
                }

                if (_currentAction != null)
                {
                    if (!_currentAction.IsInit())
                    {
                        Logger.LogInformation("Initialising {Action}", _currentAction.GetType().Name);
                        _currentAction.Init(_context);
                    }

                    Logger.LogInformation("Running {Action}", _currentAction.GetType().Name);
                    _currentAction.Run(_context);

                    if (_currentAction.IsFinished(_context))
                    {
                        Logger.LogInformation("Finishing {Action}", _currentAction.GetType().Name);
                        _currentAction.End(_context);
                        _currentAction = null;
                    }
                }

                lock (_contextLock)
                {
                    _context.DetectionTracker.TickMissing(_context.YoloDetections.Keys);
                    _context.YoloDetections.Clear();
                }
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "Exception in SelfdriveLoop");
            }

            await Task.Delay(100, token);
        }
    }

    private bool IsReadyToDrive() =>
        BaseRobot.Instance?.State.Mission == MissionEnum.Selfdrive &&
        BaseRobot.Instance?.State.MotionAllowed == true &&
        BaseRobot.Instance?.State.Mode == RobotModeEnum.Autonomous;

    private void ResetAction()
    {
        _currentAction = null;
    }

    /// <summary>
    /// Constructs (or re-uses) the action for the current test/goal selection.
    /// Only builds a new action when <see cref="_currentAction"/> is null.
    /// </summary>
    private void EnsureActionBuilt()
    {
        if (_currentAction != null)
            return;

        _currentAction = QualificationMode
            ? BuildQualificationAction()
            : BuildFunctionalAction();

        if (_currentAction != null)
            Logger.LogInformation("Built action {Action}", _currentAction.GetType().Name);
        else
            Logger.LogWarning("No action built for current test selection.");
    }

    private ISelfdriveAction? BuildQualificationAction() =>
        QualTest switch
        {
            QualificationTest.LineDetection_Q2 or
            QualificationTest.LaneKeeping_Q1 =>
                new LaneKeepAction(SelfdriveObstacles.Stopsign, ObstacleStopDistance),

            QualificationTest.LeftTurn_Q3 =>
                new SequentialAction([
                    new TimedDriveAction(ForwardSpeed, LinearVelocityUnit.MetersPerSecond.Of(0), TurnSpeed,  5_000),
                    new LaneKeepAction(SelfdriveObstacles.Barrel, ObstacleStopDistance),
                ]),

            QualificationTest.RightTurn_Q4 =>
                new SequentialAction([
                    new TimedDriveAction(ForwardSpeed, LinearVelocityUnit.MetersPerSecond.Of(0), -TurnSpeed, 3_000),
                    new LaneKeepAction(SelfdriveObstacles.Barrel, ObstacleStopDistance),
                ]),

            _ => null,
        };

    private ISelfdriveAction? BuildFunctionalAction()
    {
        // Functional tests not yet implemented.
        return null;
    }
}