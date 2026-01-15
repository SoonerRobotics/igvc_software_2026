namespace igvc_csharp.Utilities;

public static class TimeUtilities
{
    /// <summary>
    /// Returns the current time in milliseconds since the Unix epoch.
    /// </summary>
    public static ulong Now()
    {
        return (ulong)DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    }
}