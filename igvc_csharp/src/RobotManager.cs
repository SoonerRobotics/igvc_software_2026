using igvc_csharp.Core;
using Microsoft.Extensions.Logging;

namespace igvc_csharp;

public class RobotManager
{
    private static readonly ILogger Logger = Logging.From<RobotManager>();

    private static CancellationTokenSource? _gCts;
    private static Task? _periodicTask;
    private static bool _shuttingDown;
    private static Robot? _robot;

    public static async Task Run()
    {
        // Create token and hook shutdowns
        _gCts = new CancellationTokenSource();
        CreateShutdownHooks(_gCts);

        // Create robot
        _robot = new Robot();
        Robot.Instance = _robot;
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

    private static async Task CreatePeriodicLoop(Robot robot, CancellationToken ct)
    {
        try
        {
            while (!ct.IsCancellationRequested)
            {
                await robot.Periodic(ct);
                await Task.Delay(Constants.PeriodicRate, ct);
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

    private static void CreateShutdownHooks(CancellationTokenSource cts)
    {
        Console.CancelKeyPress += (_, e) =>
        {
            if (_shuttingDown)
            {
                return;
            }

            _shuttingDown = true;

            Logger.LogWarning("Ctrl + C received, shutting down");
            e.Cancel = true;
            cts.Cancel();
        };

        AppDomain.CurrentDomain.ProcessExit += (_, _) =>
        {
            if (_shuttingDown)
            {
                return;
            }

            _shuttingDown = true;

            Logger.LogWarning("Process exit signal received");
            cts.Cancel();
        };
    }
}