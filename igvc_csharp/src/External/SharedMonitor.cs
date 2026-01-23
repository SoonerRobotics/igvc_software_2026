namespace igvc_csharp.External;

using System;

public enum SensorHealth
{
    Healthy,
    Stale,
    Dead
}

public class SharedMonitor
{
    private long _lastSequence = -1;

    public SensorHealth Update(FrameHeader header)
    {
        var writeTime =
            DateTime.UnixEpoch.AddTicks(header.LastWriteTicks);

        var age = DateTime.UtcNow - writeTime;

        if (header.Sequence == _lastSequence)
            return age > TimeSpan.FromSeconds(2)
                ? SensorHealth.Dead
                : SensorHealth.Stale;

        _lastSequence = header.Sequence;

        return age > TimeSpan.FromMilliseconds(500)
            ? SensorHealth.Stale
            : SensorHealth.Healthy;
    }
}
