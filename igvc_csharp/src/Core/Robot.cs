namespace igvc_csharp.Core;

using System;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;

public sealed class Robot : BaseRobot
{
    private static readonly ILogger Logger = Logging.From<Robot>();

    public static Robot Instance { get; internal set; } = null!;

    public RobotState State { get; } = new();

    public override async Task Init(CancellationToken token)
    {
        await base.Init(token);
    }

    public override async Task Periodic(CancellationToken token)
    {
        await base.Periodic(token);
        
        // Do other stuff
    }

    public override async Task Shutdown()
    {
        await base.Shutdown();
    }

    public new void Dispose()
    {
        base.Dispose();
    }
}
