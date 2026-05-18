namespace igvc_csharp.Core;

using System;

[AttributeUsage(AttributeTargets.Class, Inherited = false)]
public sealed class SubsystemAttribute(string name) : Attribute
{
    public string Name { get; } = name;

    /// <summary>
    /// If true, this subsystem is completely ignored.
    /// </summary>
    public bool Disabled { get; init; }
}
