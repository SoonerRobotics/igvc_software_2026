using igvc_csharp.Core;
using Microsoft.Extensions.Logging;

namespace igvc_csharp;

public class RobotManager
{
    private static readonly ILogger Logger = Logging.From<RobotManager>();

    private static CancellationTokenSource? _gCts;
    private static Task? _periodicTask;
    private static bool _shuttingDown;
    private static bool _initialized;

    private static Robot? _robot;
    public static async Task InitAsync()
    {
        if (_initialized)
        {
            throw new InvalidOperationException("RobotManager is already initialized");
        }

        _initialized = true;

        // Setup shutdown hooks and token
        _gCts = new CancellationTokenSource();
        CreateShutdownHooks(_gCts);
        
        // Create the robot
        _robot = new Robot();
        Robot.Instance = _robot;
        await _robot.Init(_gCts.Token);
        
        // Loop
        _periodicTask = CreatePeriodicLoop(_robot, _gCts.Token);
    }

    public static async Task DisposeAsync()
    {
        if (!_initialized)
        {
            return;
        }
        
        // Cancel the context
        if (_gCts != null)
        {
            await _gCts.CancelAsync();
        }

        if (_periodicTask != null)
        {
            try
            {
                await _periodicTask;
            }
            catch (Exception e)
            {
                // ignored
            }
        }

        if (_robot != null)
        {
            try
            {
                await _robot.Shutdown();
            }
            catch (Exception e)
            {
                // ignored
            }

            _robot.Dispose();
            _robot = null;
        }

        if (_gCts != null)
        {
            _gCts.Dispose();
            _gCts = null;
        }
        
        Logging.Shutdown();
        _initialized = false;
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