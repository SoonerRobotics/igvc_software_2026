namespace igvc_csharp.Subsystems;

using System.Threading;
using System.Threading.Tasks;

public interface ISubsystem
{
    Task Init(CancellationToken token);
    Task Periodic(CancellationToken token);
    Task Shutdown();
    Task Restart();
}
