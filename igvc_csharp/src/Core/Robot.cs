namespace igvc_csharp.Core;

using igvc_csharp.Core.Units;
using Microsoft.Extensions.Logging;

public sealed class Robot : BaseRobot
{
    private static readonly ILogger Logger = Logging.From<Robot>();

    /// <summary>
    /// A global instance of the robot.
    /// </summary>
    public static Robot Instance { get; internal set; } = null!;

    /// <summary>
    /// The current state of the robot.
    /// </summary>
    public RobotState State { get; } = new();
    
    // Other Information

    /// <summary>
    /// The current LatLng position of the robot. Null if GPS is lost.
    /// </summary>
    public LatLng? Location { get; set; } = null;
    
    /// <summary>
    /// The current heading of the robot (e.g. absolute compass direction it is facing). Null if GPS is lost.
    /// </summary>
    public Angle? Heading { get; set; } = null;

    public new void Dispose()
    {
        base.Dispose();
    }
}
