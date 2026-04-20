namespace igvc_csharp.Core;

public sealed class SubsystemProperty<T> : AbstractSubsystemProperty
{
    private T? value;

    public SubsystemProperty(string key) : base(key)
    {
    }

    public SubsystemProperty(string key, T def) : base(key)
    {
        value = def;
    }

    public void Set(T? val)
    {
        value = val;
        parent.OnPropertyUpdated(key, val);
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
