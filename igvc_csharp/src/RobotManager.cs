using igvc_csharp.Core;
using Microsoft.Extensions.Logging;

namespace igvc_csharp;

public class RobotManager
{
    private static readonly ILogger Logger = Logging.From<RobotManager>();

    private static CancellationTokenSource? _gCts;
    private static Task? _periodicTask;
    private static bool _shuttingDown;
    private static IgvcRobot? _robot;

    public static async Task Run()
    {
        // Create token and hook shutdowns
        _gCts = new CancellationTokenSource();
        CreateShutdownHooks(_gCts);

        // Create robot
        _robot = new IgvcRobot();
        await _robot.Init(_gCts.Token);

        // Create periodoic task
        _periodicTask = CreatePeriodicLoop(_robot, _gCts.Token);

        // Wait for program to shut down
        try
        {
            await Task.Delay(Timeout.Infinite, _gCts.Token);
        }
        catch (OperationCanceledException)
        {
            // ignore
        }

        // Kill it all
        try
        {
            await _periodicTask;
        }
        catch (OperationCanceledException)
        {
        }

        await _robot.Shutdown();
        _robot.Dispose();
        _gCts.Dispose();
        Logging.Shutdown();
    }

    private static async Task CreatePeriodicLoop(IgvcRobot robot, CancellationToken ct)
    {
        try
        {
            while (!ct.IsCancellationRequested)
            {
                await robot.Periodic(ct);
                await Task.Delay(Configuration.PeriodicRate, ct);
            }
        }
        catch (OperationCanceledException)
        {
            // We expect this on shutdown
        }
        catch (Exception e)
        {
            Logger.LogCritical("Critical exception in periodic loop");
            throw;
        }
    }

    public static void Shutdown()
    {
        if (_shuttingDown)
        {
            return;
        }

        _shuttingDown = true;

        Logger.LogInformation("Shutdown initiated");
        _gCts?.Cancel();
    }

    private static void CreateShutdownHooks(CancellationTokenSource cts)
    {
        Console.CancelKeyPress += (_, e) =>
        {
            Logger.LogInformation("Shutdown initiated via console cancel key press");
            e.Cancel = true; // prevent the process from being killed immediately
            cts.Cancel();
        };

        AppDomain.CurrentDomain.ProcessExit += (_, _) =>
        {
            Logger.LogInformation("Shutdown initiated via process exit signal");
            Shutdown();
        };
    }
}