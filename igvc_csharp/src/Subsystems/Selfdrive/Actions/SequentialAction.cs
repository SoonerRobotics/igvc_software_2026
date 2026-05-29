
using igvc_csharp.src.selfdrive.actions;
using igvc_csharp.src.Subsystems.selfdrive;

namespace igvc_csharp.src.subsystems.selfdrive;

/*
TODO: document this and like, link to WPILib and stuff
*/
public class SequentialAction(List<ISelfdriveAction> actions) : ISelfdriveAction
{
    private List<ISelfdriveAction> _actions = actions;
    private int _index = 0;

    public bool IsInit()
    {
        if (_actions.Count == 0)
        {
            return true;
        }
        return _actions[0].IsInit();
    }

    public void Init(SelfdriveContext context)
    {
        if (_actions.Count > 0)
        {
            _actions[0].Init(context);
        }
    }

    public void Run(SelfdriveContext context)
    {
        if (_index < (_actions.Count - 1) && _actions[_index].IsFinished(context))
        {
            _actions[_index].End(context);
            _index++;
            _actions[_index].Init(context);
        }

        _actions[_index].Run(context);
    }

    public void End(SelfdriveContext context)
    {
        if (_actions.Count > 0)
        {
            _actions[_index].End(context);
        }
        //FIXME do we need to do something else here?
    }

    public bool IsFinished(SelfdriveContext context)
    {
        if (_actions.Count == 0)
        {
            return true;
        }
        return _actions[-1].IsFinished(context);
    }
}