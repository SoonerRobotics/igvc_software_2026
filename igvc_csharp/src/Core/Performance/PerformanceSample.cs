namespace igvc_csharp.Core.Performance;

public readonly record struct PerformanceSample(
    string Subsystem,
    string Group,
    string Name,
    string Unit,
    MetricAggregate Aggregate,
    DateTime Timestamp,
    double Value
);