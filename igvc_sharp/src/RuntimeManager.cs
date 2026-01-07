using Microsoft.Extensions.Logging;

namespace igvc_sharp;

public static class RuntimeManager
{
    private static readonly ILogger Logger = Logging.ForContext<Robot>();
    private static readonly CancellationTokenSource Cts = new();
    private static readonly TaskCompletionSource<bool> ShutdownTcs = new();
    private static int _shutdownLock;
    private static Task? _periodicTask;

    public static async Task Begin()
    {
        Logger.LogInformation("Starting RuntimeManager");

        await Robot.Instance.Init();

        _periodicTask = StartPeriodic(Cts.Token);

        RegisterShutdownHooks();

        // Wait here until shutdown is triggered
        await ShutdownTcs.Task;
    }

    private static async Task StartPeriodic(CancellationToken token)
    {
        using var timer = new PeriodicTimer(Constants.PeriodicRate);

        while (await timer.WaitForNextTickAsync(token))
        {
            // Ensure Periodic() never overlaps
            await Robot.Instance.Periodic();
        }
    }

    public static async Task Shutdown()
    {
        if (Interlocked.Exchange(ref _shutdownLock, 1) == 1)
        {
            // We are already shutting down
            return;
        }

        Logger.LogInformation("Shutting down RuntimeManager");

        await Cts.CancelAsync();

        if (_periodicTask is not null)
        {
            try
            {
                await _periodicTask;
            }
            catch (OperationCanceledException)
            {
            }
        }

        await Robot.Instance.Shutdown();

        Logger.LogInformation("Shutdown complete");

        ShutdownTcs.TrySetResult(true);
    }


    private static void RegisterShutdownHooks()
    {
        Console.CancelKeyPress += (_, e) =>
        {
            e.Cancel = true; // prevent immediate termination
            _ = Shutdown();
        };

        AppDomain.CurrentDomain.ProcessExit += (_, _) => { _ = Shutdown(); };
    }
}