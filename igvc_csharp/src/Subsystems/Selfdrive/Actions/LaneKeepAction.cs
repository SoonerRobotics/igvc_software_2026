

public class LaneKeepAction(Lane lane, Until condition) : SelfdriveAction
{
    public override void Init(SelfdriveContext context)
    {
        //TODO: make an A* and pure pursuit
    }

    public override void Run(SelfdriveContext context)
    {
        //TODO: calculate motor outputs to keep us centered in current lane
    }

    public override void End(SelfdriveContext context)
    {
        //TODO: do nothing??? Mat.Dispose()?
    }

    public override bool IsFinished(SelfdriveContext context)
    {
        return condition == true;
    }
}