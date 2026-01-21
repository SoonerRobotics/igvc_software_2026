namespace igvc_csharp.Utils;

public static class TimeUtils
{
    /// <summary>
    /// Returns the current time in milliseconds since the Unix epoch.
    /// </summary>
    public static ulong Now()
    {
        return (ulong)DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    }
}