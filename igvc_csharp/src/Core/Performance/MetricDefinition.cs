namespace igvc_csharp.Core.Performance;

public readonly record struct MetricDefinition(
    string Subsystem,
    string Group,
    string Name,
    string Unit,
    MetricAggregate Aggregate
);