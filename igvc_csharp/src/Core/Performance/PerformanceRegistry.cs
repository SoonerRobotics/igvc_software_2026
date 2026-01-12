using igvc_csharp.Events;

namespace igvc_csharp.Core.Performance;

using System.Collections.Concurrent;

public sealed class PerformanceRegistry
{
    public static PerformanceRegistry Instance { get; } = new();

    private readonly ConcurrentDictionary<string, IPerformanceMetric> _metrics = new();

    private PerformanceRegistry()
    {
    }

    public void Register(IPerformanceMetric metric)
    {
        var key = $"{metric.Definition.Subsystem}:{metric.Definition.Group}:{metric.Definition.Name}";
        _metrics[key] = metric;
    }

    internal void Emit<T>(PerformanceMetric<T> metric, DateTime ts) where T : struct
    {
        var sample = metric.GetAggregatedSample(ts);
        EventBus.Instance.Publish(new PerformanceSampleEvent(sample));
    }

    public IReadOnlyList<MetricDefinition> GetSchema()
        => _metrics.Values.Select(m => m.Definition).ToList();

    public IReadOnlyList<PerformanceSample> GetAllHistory()
        => _metrics.Values.SelectMany(m => m.GetHistory()).ToList();
}