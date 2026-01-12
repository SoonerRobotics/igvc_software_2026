namespace igvc_csharp.Core.Performance;

[AttributeUsage(AttributeTargets.Field)]
public sealed class MetricAttribute : Attribute
{
    public string Name { get; }
    public string Unit { get; }

    public string Group { get; set; } = "Default";
    public MetricAggregate Aggregate { get; set; } = MetricAggregate.Last;

    public int MaxSamples { get; set; } = 300;
    public int MaxAgeSeconds { get; set; } = 0;

    public int EmitEveryMs { get; set; } = 0; // 0 = every update

    public MetricAttribute(string name, string unit)
    {
        Name = name;
        Unit = unit;
    }
}