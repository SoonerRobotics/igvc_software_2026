using igvc_csharp.Events;

namespace igvc_csharp.src.Subsystems.selfdrive;

public class TrackedDetection
{
    public YoloDetectionEvent LastKnown { get; set; }
    public int ConfirmFrames { get; set; }   // frames seen consecutively
    public int LostFrames { get; set; }      // frames missing consecutively
    public bool IsConfirmed { get; set; }

    public TrackedDetection(YoloDetectionEvent e)
    {
        LastKnown = e;
        ConfirmFrames = 1;
        LostFrames = 0;
        IsConfirmed = false;
    }
}

public class YoloDetectionTracker
{
    private readonly int _framestoConfirm;
    private readonly int _framesToLose;
    private readonly Dictionary<string, TrackedDetection> _tracked = new();

    public YoloDetectionTracker(int framesToConfirm = 3, int framesToLose = 5)
    {
        _framestoConfirm = framesToConfirm;
        _framesToLose = framesToLose;
    }

    /// <summary>
    /// Call this each time a raw detection arrives from YOLO.
    /// </summary>
    public void Feed(YoloDetectionEvent e)
    {
        if (_tracked.TryGetValue(e.label, out var tracked))
        {
            tracked.LastKnown = e;
            tracked.LostFrames = 0;
            tracked.ConfirmFrames++;

            if (tracked.ConfirmFrames >= _framestoConfirm)
                tracked.IsConfirmed = true;
        }
        else
        {
            _tracked[e.label] = new TrackedDetection(e);
        }
    }

    /// <summary>
    /// Call once per update tick for every label that was NOT seen this frame.
    /// Or just call TickAll() after feeding all detections.
    /// </summary>
    public void TickMissing(IEnumerable<string> seenLabels)
    {
        var seen = new HashSet<string>(seenLabels);
        foreach (var (label, tracked) in _tracked)
        {
            if (seen.Contains(label)) continue;

            tracked.ConfirmFrames = 0;
            tracked.LostFrames++;

            if (tracked.LostFrames >= _framesToLose)
                tracked.IsConfirmed = false;
        }

        // Clean up entries that have been lost long enough
        var toRemove = _tracked
            .Where(kv => kv.Value.LostFrames >= _framesToLose)
            .Select(kv => kv.Key)
            .ToList();
        foreach (var key in toRemove)
            _tracked.Remove(key);
    }

    /// <summary>
    /// Returns confirmed detections only — use this instead of raw YoloDetections.
    /// </summary>
    public bool TryGetConfirmed(string label, out YoloDetectionEvent e)
    {
        e = default!;
        if (_tracked.TryGetValue(label, out var tracked) && tracked.IsConfirmed)
        {
            e = tracked.LastKnown;
            return true;
        }
        return false;
    }
}