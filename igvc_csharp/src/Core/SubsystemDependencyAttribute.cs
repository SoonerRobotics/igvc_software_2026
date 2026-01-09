namespace igvc_csharp.Core;

[AttributeUsage(AttributeTargets.Parameter)]
public class SubsystemDependencyAttribute : Attribute
{
    /// <summary>
    /// Whether or not this subsystem dependency is required.
    /// </summary>
    public bool Required { get; init; }
}