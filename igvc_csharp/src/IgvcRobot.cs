namespace igvc_csharp.Core;

using igvc_csharp.Core.Units;
using igvc_csharp.Utils;
using Microsoft.Extensions.Logging;

public sealed class IgvcRobot : BaseRobot
{
    private static readonly ILogger Logger = Logging.From<IgvcRobot>();

    public override async Task Init(CancellationToken token)
    {
        // Handle default stuff
        await base.Init(token);
    }

    public new void Dispose()
    {
        base.Dispose();
    }
}
