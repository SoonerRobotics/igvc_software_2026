using Microsoft.Extensions.Logging;

namespace igvc_sharp;

public static class Constants
{
    public const LogLevel DefaultLogLevel = LogLevel.Trace;
    public static readonly TimeSpan PeriodicRate = TimeSpan.FromMilliseconds(1000 / 20); // 20hz
}