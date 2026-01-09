namespace igvc_csharp.Core;

using System;

[AttributeUsage(AttributeTargets.Class, Inherited = false, AllowMultiple = false)]
public sealed class SubsystemAttribute : Attribute
{
    public string Name { get; }
    
    /// <summary>
    /// If true, this subsystem is completely ignored.
    /// </summary>
    public bool Disabled { get; init; }

    /// <summary>
    /// Other subsystems this subsystem relies on.
    /// These must be enabled and created for this subsystem to be created.
    /// </summary>
    public Type[] DependsOn { get; init; } = [];

    public SubsystemAttribute(string name)
    {
        Name = name;
    }
}
