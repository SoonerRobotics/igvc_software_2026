
namespace igvcsharp.selfdrive.actions;

public class TurnAction(direction dir, ulong timeout) : SelfdriveAction
{
    private direction _dir = dir;
    private ulong _timeout = timeout;
    private ulong _startTime;

    public override void Init(SelfdriveContext context)
    {
        _startTime = TimeUtils.Now();
    }

    public override void Run(SelfdriveContext context)
    {
        if (_dir == left)
        {
            //TODO: turn left
        }
        else
        {
            //TODO: turn right
        }
    }

    public override void End(SelfdriveContext context)
    {
        //TODO: do nothing??? Mat.Dispose()?
    }

    public override bool IsFinished(SelfdriveContext context)
    {
        return (TimeUtils.Now() - _startTime) > _timeout;
    }
}