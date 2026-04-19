namespace igvc_csharp.Core;

public record ProcessManagerConfig
{
    public string Arguments { get; init; } = string.Empty;
    public bool AutoRestart { get; init; } = true;
    public int MaxRestartAttempts { get; init; } = 5;
    public int RestartDelayMs { get; init; } = 2000;
    public int CrashThresholdMs { get; init; } = 3000;
    public int GracefulShutdownTimeoutMs { get; init; } = 5000;
    public int MaxJsonParseErrorLogs { get; init; } = 10;
}