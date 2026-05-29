
namespace igvc_csharp.src.Subsystems.selfdrive.actions;

public class LaneKeepAction(SelfdriveMachine.SelfdriveLane lane, SelfdriveMachine.SelfdriveObstacle obstacle, int timeout = -1) : SelfdriveAction
{
    private int _startTime = -1;
    
    public override void Init(SelfdriveContext context)
    {
        _startTime = TimeUtils.Now();

        //TODO: make an A* and pure pursuit
    }

    public override void Run(SelfdriveContext context)
    {
        //TODO: calculate motor outputs to keep us centered in current lane

        canbus.MotorControl.SetVelocities(0, 0, 0); //FIXME
    }

    public override void End(SelfdriveContext context)
    {
        //TODO: do nothing??? Mat.Dispose()?

        //FIXME we could have like a "StopOnEnd" field in the constructor?
        canbus.MotorControl.SetVelocities(0, 0, 0);
    }

    public override bool IsFinished(SelfdriveContext context)
    {
        if (_startTime == -1)
        {
            return false;
        }
        else if ((TimeUtils.Now() - _startTime) > timeout)
        {
            return true;
        }
        else
        {
            return false; //FIXME detect if there's an obstacle within 3 feet of us
        }
    }
}