using Messages.Arc;

namespace igvc_csharp.Subsystems.Arc;

[AttributeUsage(AttributeTargets.Method)]
public sealed class ArcCommandAttribute(ArcCommandId command) : Attribute
{
    public ArcCommandId Command { set; get; } = command;
}