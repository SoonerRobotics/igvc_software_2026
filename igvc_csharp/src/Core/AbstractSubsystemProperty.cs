namespace igvc_csharp.Core;

public abstract class AbstractSubsystemProperty(string key)
{
    protected string key = key;
    protected SubsystemBase? parent { get; } //FIXME find some way to like, actually set this ???
}
