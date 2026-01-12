namespace igvc_csharp.Core.Performance;

public interface IPerformanceMetric
{
    MetricDefinition Definition { get; }
    IEnumerable<PerformanceSample> GetHistory();
}