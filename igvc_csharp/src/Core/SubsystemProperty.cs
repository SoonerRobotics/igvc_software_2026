namespace igvc_csharp.Core;

public sealed class SubsystemProperty<T>
{
    private T? value;
    private string key;

    public SubsystemProperty(string key)
    {
        this.key = key;
    }

    public SubsystemProperty(string key, T def)
    {
        value = def;
        this.key = key;
    }

    public void Set(T? val)
    {
        value = val;
        // TODO: Broadcast
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
