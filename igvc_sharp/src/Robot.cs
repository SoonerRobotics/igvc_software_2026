using Microsoft.Extensions.Logging;

namespace igvc_sharp;

public class Robot
{
    private static readonly ILogger Logger = Logging.ForContext<Robot>();
    public static Robot Instance = new ();
    
    /// <summary>
    /// This method is called whenever the <c>Robot</c> is started/initialized.
    /// </summary>
    public async Task Init()
    {
        Logger.LogInformation("Initializing");
    }

    /// <summary>
    /// This method is called every <c>Constants.PeriodicRate</c> milliseconds.
    /// </summary>
    public async Task Periodic()
    {

    }

    /// <summary>
    /// This method is called whenever the <c>Robot</c> is shutdown.
    /// </summary>
    public async Task Shutdown()
    {
        Logger.LogInformation("Shutting down");
    }
}