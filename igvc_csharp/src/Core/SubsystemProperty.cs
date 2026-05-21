namespace igvc_csharp.Core;

public struct SubsystemPropertyUpdate<T>
{
    public string Key { get; }
    public T? Value { get; }
    public DateTime Timestamp { get; } = DateTime.UtcNow;

    public SubsystemPropertyUpdate(string key, T? value)
    {
        Key = key;
        Value = value;
    }
}

public sealed class SubsystemProperty<T> : AbstractSubsystemProperty
{
    private T? value;
    private Queue<SubsystemPropertyUpdate<T>> history = new();

    public SubsystemProperty(string key) : base(key)
    {
        
    }
    
    public SubsystemProperty(string key, int maxHistory) : base(key)
    {
        history = new Queue<SubsystemPropertyUpdate<T>>(maxHistory);
    }

    public SubsystemProperty(string key, T def) : base(key)
    {
        value = def;
    }

    public void Set(T? val)
    {
        value = val;
        history.Enqueue(new SubsystemPropertyUpdate<T>(key, val));
        parent?.OnPropertyUpdated(key, val); //FIXME we need to make parent not null or something I think?
    }

    public T? Get()
    {
        return value;
    }

    public T? Get(T def)
    {
        return value == null ? value : def;
    }
}
