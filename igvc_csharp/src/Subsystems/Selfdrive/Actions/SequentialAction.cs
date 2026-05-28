

/*
TODO: document this and like, link to WPILib and stuff
*/
public class SequentialAction : Action
{
    private List<Action> _actions = new();
    private int _index = 0;
    private SelfdriveContext _ctx;

    public SequentialAction(List<Action> actions) //FIXME make this like, variable arguments list?
    {
        _actions = actions;
    }

    public override void Init(SelfdriveContext context)
    {
        _actions[0].Init(context);
    }

    public override void Run(SelfdriveContext context)
    {
        if (_actions[_index].IsFinished(context) && _index < _actions.Length-1)
        {
            _actions[_index].End(context);
            _index++;
            _actions[_index].Init(context);
        }

        _actions[_index].Run(context);
    }

    public override void End(context)
    {
        _actions[index].End(context);
        //FIXME do we need to do something else here?
    }

    public override boolean IsFinished(SelfdriveContext context)
    {
        return _actions[-1].IsFinished(context);
    }
}