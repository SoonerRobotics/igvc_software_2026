namespace igvc_csharp.Subsystems.Arc;

[AttributeUsage(AttributeTargets.Method)]
public sealed class ArcCommandAttribute(Command command) : Attribute
{
    public Command Command { set; get; } = command;
}