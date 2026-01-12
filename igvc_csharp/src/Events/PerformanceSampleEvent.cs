using igvc_csharp.Core;
using igvc_csharp.Core.Performance;

namespace igvc_csharp.Events;

public readonly record struct PerformanceSampleEvent(
    PerformanceSample Sample
) : IRobotEvent;