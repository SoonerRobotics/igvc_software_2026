namespace igvc_csharp.Core;

using System;

[AttributeUsage(AttributeTargets.Class, Inherited = false, AllowMultiple = false)]
public sealed class SubsystemAttribute : Attribute
{
    public string Name { get; }
    public bool Disabled { get; init; }

    public SubsystemAttribute(string name)
    {
        Name = name;
    }
}
