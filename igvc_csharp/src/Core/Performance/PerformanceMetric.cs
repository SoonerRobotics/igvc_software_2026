namespace igvc_csharp.Core.Performance;

// ReSharper disable once ClassNeverInstantiated.Global
public sealed class PerformanceMetric<T>(
    MetricDefinition definition,
    int maxSamples,
    TimeSpan? maxAge,
    int emitEveryMs)
    : IPerformanceMetric
    where T : struct
{
    private readonly LinkedList<(DateTime ts, double value)> _samples = new();
    private readonly object _lock = new();

    private DateTime _lastEmit = DateTime.MinValue;

    public MetricDefinition Definition { get; } = definition;

    public void AddSample(T value)
    {
        var now = DateTime.UtcNow;
        var v = Convert.ToDouble(value);

        lock (_lock)
        {
            _samples.AddLast((now, v));

            if (maxAge.HasValue)
            {
                while (_samples.First != null && now - _samples.First.Value.ts > maxAge.Value)
                {
                    _samples.RemoveFirst();
                }
            }
            else
            {
                while (_samples.Count > maxSamples)
                {
                    _samples.RemoveFirst();
                }
            }

            if (emitEveryMs > 0 && (now - _lastEmit).TotalMilliseconds < emitEveryMs)
            {
                return;
            }

            _lastEmit = now;
        }

        PerformanceRegistry.Instance.Emit(this, now);
    }

    public IEnumerable<PerformanceSample> GetHistory()
    {
        lock (_lock)
        {
            foreach (var (ts, value) in _samples)
            {
                yield return new PerformanceSample(
                    Definition.Subsystem,
                    Definition.Group,
                    Definition.Name,
                    Definition.Unit,
                    Definition.Aggregate,
                    ts,
                    value);
            }
        }
    }

    internal PerformanceSample GetAggregatedSample(DateTime ts)
    {
        lock (_lock)
        {
            if (_samples.Count == 0)
                return default;

            var value = Definition.Aggregate switch
            {
                MetricAggregate.Average => _samples.Average(s => s.value),
                MetricAggregate.Min => _samples.Min(s => s.value),
                MetricAggregate.Max => _samples.Max(s => s.value),
                _ => _samples.Last!.Value.value
            };
            return new PerformanceSample(
                Definition.Subsystem,
                Definition.Group,
                Definition.Name,
                Definition.Unit,
                Definition.Aggregate,
                ts,
                value
            );
        }
    }
}