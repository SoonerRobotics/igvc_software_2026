
namespace igvc_csharp.src.selfdrive.actions;

public interface ISelfdriveAction()
{
    public void Init(SelfdriveContext context);
    public void Run(SelfdriveContext context);
    public void End(SelfdriveContext context);
    public boolean IsFinished(SelfdriveContext context);
}
