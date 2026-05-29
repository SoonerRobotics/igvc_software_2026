
namespace igvc_csharp.src.Subsystems.selfdrive.actions;

public class TurnAction(SelfdriveMachine.SelfdriveLane direction, ulong timeout) : SelfdriveAction
{
    private int _startTime = -1;

    public override void Init(SelfdriveContext context)
    {
        _startTime = TimeUtils.Now();
    }

    public override void Run(SelfdriveContext context)
    {
        if (direction == SelfdriveMachine.SelfdriveLane.Left)
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