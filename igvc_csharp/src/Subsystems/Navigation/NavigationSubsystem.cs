using igvc_csharp.Core;
using igvc_csharp.Core.Config;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Subsystems.Navigation;

/// <summary>
/// Consumes the inflated binary image published by VisionSubsystem,
/// runs the frontier BFS + A* pipeline, and re-publishes:
///   - a debug image showing the path overlaid on the inflated map
///   - (future) the path for PathResolverSubsystem to follow
///
/// Pipeline per frame
/// ──────────────────
///   inflated BGR image
///       │
///       ▼
///   extract occupancy grid  (white px → obstacle, scale to 0-255)
///       │
///       ▼
///   AStarPlanner.FindPath   (frontier BFS goal + A*)
///       │
///       ▼
///   BuildDebugImage         (grid + path overlay)
///       │
///       ▼
///   publish "nav_debug" ImageFrame + Path (metric waypoints via GridToLocal)
/// </summary>
[Subsystem("NavigationSubsystem", Disabled = false)]
public class NavigationSubsystem : SubsystemBase
{
    private readonly AStarConfig _config = new();
    private readonly PurePursuit _pursuit = new();

    // Last computed path in grid coords — exposed for future PathResolver use
    public IReadOnlyList<(int x, int y)>? LastGridPath { get; private set; }

    // Metric path in robot-local coords (x = forward, y = left)
    public IReadOnlyList<(float x, float y)>? LastLocalPath { get; private set; }

    public override Task Init(CancellationToken token)
    {
        // Subscribe to the inflated image that VisionSubsystem publishes
        SubscribeImage("combined_inflated", OnInflatedImageReceived, token);

        SetOperatingState(SubsystemState.Operating);
        return Task.CompletedTask;
    }

    private Task OnInflatedImageReceived(ImageFrame frame, CancellationToken token)
    {
        try
        {
            ProcessFrame(frame);
        }
        catch (Exception ex)
        {
            Logger.LogError(ex, "Navigation processing failed");
        }
        return Task.CompletedTask;
    }

    private void ProcessFrame(ImageFrame frame)
    {
        // ── 1. Decode image ──────────────────────────────────────────────────
        using var mat = CvUtils.AsMat(frame);

        // Resize to planning grid if the image dimensions differ
        using var resized = new Mat();
        if (mat.Width != _config.GridWidth || mat.Height != _config.GridHeight)
            Cv2.Resize(mat, resized, new Size(_config.GridWidth, _config.GridHeight),
                       interpolation: InterpolationFlags.Nearest);
        var src = resized.Empty() ? mat : resized;

        // ── 2. Build occupancy grid ──────────────────────────────────────────
        // White pixels (obstacles after ThresholdFilter + InflationFilter) → 255
        // Black (free space) → 0
        // A* treats values >= ObstacleThreshold (50) as impassable.
        using var gray = new Mat();
        if (src.Channels() > 1)
            Cv2.CvtColor(src, gray, ColorConversionCodes.BGR2GRAY);
        else
            src.CopyTo(gray);

        int W = _config.GridWidth;
        int H = _config.GridHeight;
        var costMap = new byte[W * H];
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                costMap[x + W * y] = gray.At<byte>(y, x);

        // ── 3. Plan ──────────────────────────────────────────────────────────
        var gridPath = AStarPlanner.FindPath(costMap, _config);

        if (gridPath is null || gridPath.Count == 0)
        {
            Logger.LogWarning("A* found no path — open space may be fully blocked");
            return;
        }

        // Store for external consumers
        LastGridPath = gridPath;

        // Convert to metric and feed PurePursuit
        var localPath = gridPath
            .Select(cell => AStarPlanner.GridToLocal(cell, _config))
            .ToList();
        LastLocalPath = localPath;
        _pursuit.SetPoints(localPath);

        // ── 4. Build and publish debug image ─────────────────────────────────
        var goal = gridPath[^1];
        using var debugMat = AStarPlanner.BuildDebugImage(costMap, gridPath, goal, _config, displayScale: 4);

        // Overlay the path on the inflated source image as a second view
        using var overlayMat = OverlayPathOnSource(src, gridPath, goal);

        PublishImage(debugMat, "nav_astar_debug");
        PublishImage(overlayMat, "nav_path_overlay");
    }

    /// <summary>
    /// Draws the planned path and goal directly on top of the (possibly resized)
    /// inflated image so operators can see path vs. raw obstacles together.
    /// </summary>
    private static Mat OverlayPathOnSource(
        Mat src,
        List<(int x, int y)> path,
        (int x, int y) goal)
    {
        var overlay = src.Channels() == 1
            ? src.CvtColor(ColorConversionCodes.GRAY2BGR)
            : src.Clone();

        foreach (var (px, py) in path)
            Cv2.Circle(overlay, new Point(px, py), 1, new Scalar(0, 255, 0), -1);

        Cv2.Circle(overlay, new Point(goal.x, goal.y), 3, new Scalar(255, 80, 0), -1);
        Cv2.Circle(overlay, new Point(overlay.Width / 2, overlay.Height - 2), 3,
                   new Scalar(0, 0, 255), -1);

        return overlay;
    }

    /// <summary>
    /// Publishes a Mat as a named ImageFrame on the EventBus.
    /// </summary>
    private void PublishImage(Mat mat, string name)
    {
        var bytes = CvUtils.FromMat(mat);
        var imgFrame = MessageConstructor.CreateImageFrame(
            (uint)mat.Width, (uint)mat.Height, name, bytes);
        EventBus.Instance.Publish(new MessageWrapperEvent(
            MessageWrapper.From(MessageType.ImageFrame, imgFrame.ByteBuffer.ToFullArray())));
    }
}