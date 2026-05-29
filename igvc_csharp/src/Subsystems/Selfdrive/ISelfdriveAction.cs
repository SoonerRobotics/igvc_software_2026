
using igvc_csharp.src.Subsystems.selfdrive;

namespace igvc_csharp.src.selfdrive.actions;

public interface ISelfdriveAction
{
    public bool IsInit();
    public void Init(SelfdriveContext context);
    public void Run(SelfdriveContext context);
    public void End(SelfdriveContext context);
    public bool IsFinished(SelfdriveContext context);
}
