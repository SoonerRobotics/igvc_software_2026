using OpenCvSharp;

namespace igvc_csharp.Subsystems.Navigation;

public record AStarConfig
{
    public int GridWidth { get; init; } = 80;
    public int GridHeight { get; init; } = 80;

    public float HorizontalFov { get; init; } = 3.4f;
    public float VerticalFov { get; init; } = 2.75f;

    public float DepthWeight { get; init; } = 2.2f;
    public float ForwardWeight { get; init; } = 1.3f;

    public int MaxFrontierDepth { get; init; } = 50;

    public byte ObstacleThreshold { get; init; } = 50;

    public float WaypointWeight { get; init; } = 1.0f;
    public float WaypointMaxWeight { get; init; } = 10.0f;

    /// <summary>
    /// Milliseconds to wait after entering autonomous before waypoints influence path planning.
    /// </summary>
    public ulong WaypointDelayMs { get; init; } = 8000;

    /// <summary>
    /// When true, the cost map is zeroed out and the robot steers purely by waypoint heading.
    /// </summary>
    public bool UseOnlyWaypoints { get; init; } = false;
}

public static class AStarPlanner
{
    private static readonly (int dx, int dy, float cost)[] SearchDirs =
        Enumerable.Range(-1, 3)
            .SelectMany(dx => Enumerable.Range(-1, 3)
                .Where(dy => !(dx == 0 && dy == 0))
                .Select(dy => (dx, dy, MathF.Sqrt(dx * dx + dy * dy))))
            .ToArray();

    public static List<(int x, int y)>? FindPath(
        byte[] costMap,
        AStarConfig config,
        float? waypointHeadingRad = null,
        float? robotThetaRad = null)
    {
        int W = config.GridWidth;
        int H = config.GridHeight;

        var robotPos = (x: W / 2, y: H - 2);
        var goalPos = FindBestGoal(costMap, robotPos, config, waypointHeadingRad, robotThetaRad);

        return AStar(robotPos, goalPos, costMap, config);
    }

    private static float AngleDifference(float to, float from)
    {
        float delta = to - from;
        delta = (delta + MathF.PI) % (2 * MathF.PI) - MathF.PI;
        if (delta < -MathF.PI) delta += 2 * MathF.PI;
        return delta;
    }

    private static (int x, int y) FindBestGoal(
        byte[] costMap,
        (int x, int y) robotPos,
        AStarConfig config,
        float? waypointHeadingRad,
        float? robotThetaRad)
    {
        int W = config.GridWidth;
        int H = config.GridHeight;

        var frontier = new HashSet<(int, int)> { robotPos };
        var explored = new HashSet<int>();

        var bestPos = robotPos;
        float bestCost = float.MinValue;
        int depth = 0;

        while (depth < config.MaxFrontierDepth && frontier.Count > 0)
        {
            var current = new HashSet<(int, int)>(frontier);
            frontier.Clear();

            foreach (var (x, y) in current)
            {
                float score = (H - y) * config.ForwardWeight + depth * config.DepthWeight;

                if (waypointHeadingRad.HasValue && robotThetaRad.HasValue)
                {
                    float angleToCell = MathF.Atan2(W / 2f - x, H - 2f - y);
                    float headingToCell = robotThetaRad.Value + angleToCell;
                    float headingErr = MathF.Abs(AngleDifference(headingToCell, waypointHeadingRad.Value)) * (180f / MathF.PI);
                    score -= MathF.Max(headingErr, config.WaypointMaxWeight) * config.WaypointWeight;
                }

                if (score > bestCost)
                {
                    bestCost = score;
                    bestPos = (x, y);
                }

                explored.Add(x + W * y);

                TryAdd(x, y - 1, costMap, W, H, explored, frontier, config);
                TryAdd(x + 1, y, costMap, W, H, explored, frontier, config);
                TryAdd(x - 1, y, costMap, W, H, explored, frontier, config);
            }

            depth++;
        }

        return bestPos;
    }

    private static void TryAdd(
        int x, int y,
        byte[] costMap,
        int W, int H,
        HashSet<int> explored,
        HashSet<(int, int)> frontier,
        AStarConfig config)
    {
        if (x < 0 || x >= W || y < 0 || y >= H) return;
        int idx = x + W * y;
        if (explored.Contains(idx)) return;
        if (costMap[idx] >= config.ObstacleThreshold) return;
        frontier.Add((x, y));
    }

    private static List<(int x, int y)>? AStar(
        (int x, int y) start,
        (int x, int y) goal,
        byte[] costMap,
        AStarConfig config)
    {
        int W = config.GridWidth;
        int H = config.GridHeight;

        var closed = new HashSet<(int, int)>();
        var cameFrom = new Dictionary<(int, int), (int, int)>();

        var gScore = new Dictionary<(int, int), float> { [start] = 0f };
        var open = new SortedSet<(float f, int seq, (int x, int y) node)>(
            Comparer<(float f, int seq, (int x, int y) node)>.Create((a, b) =>
            {
                int c = a.f.CompareTo(b.f);
                return c != 0 ? c : a.seq.CompareTo(b.seq);
            })
        );

        float Heuristic((int x, int y) p) =>
            MathF.Sqrt((goal.x - p.x) * (goal.x - p.x) +
                       (goal.y - p.y) * (goal.y - p.y));

        int seq = 0;
        open.Add((Heuristic(start), seq++, start));

        while (open.Count > 0)
        {
            var (_, _, current) = open.Min;
            open.Remove(open.Min);

            if (current == goal)
                return ReconstructPath(cameFrom, current);

            if (!closed.Add(current)) continue;

            float gCurrent = gScore.GetValueOrDefault(current, float.MaxValue);

            foreach (var (dx, dy, moveCost) in SearchDirs)
            {
                var neighbor = (x: current.x + dx, y: current.y + dy);
                if (neighbor.x < 0 || neighbor.x >= W ||
                    neighbor.y < 0 || neighbor.y >= H)
                    continue;

                if (closed.Contains(neighbor)) continue;

                float cellCost = costMap[neighbor.x + W * neighbor.y] / 10f;
                float tentG = gCurrent + moveCost + cellCost;

                if (tentG < gScore.GetValueOrDefault(neighbor, float.MaxValue))
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentG;
                    open.Add((tentG + Heuristic(neighbor), seq++, neighbor));
                }
            }
        }

        return null;
    }

    private static List<(int x, int y)> ReconstructPath(
        Dictionary<(int, int), (int, int)> cameFrom,
        (int x, int y) current)
    {
        var path = new List<(int, int)> { current };
        while (cameFrom.TryGetValue(current, out var prev))
        {
            current = prev;
            path.Add(current);
        }
        path.Reverse();
        return path;
    }

    public static (float x, float y) GridToLocal((int gx, int gy) cell, AStarConfig config)
    {
        float x = (config.GridHeight - cell.gy) * config.VerticalFov / config.GridHeight;
        float y = (config.GridWidth / 2f - cell.gx) * config.HorizontalFov / config.GridWidth;
        return (x, y);
    }

    public static Mat BuildDebugImage(
        byte[] costMap,
        List<(int x, int y)>? path,
        (int x, int y) goal,
        AStarConfig config,
        int displayScale = 4)
    {
        int W = config.GridWidth;
        int H = config.GridHeight;

        using var gray = new Mat(H, W, MatType.CV_8UC1);
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                gray.At<byte>(y, x) = costMap[x + W * y];

        var img = new Mat();
        Cv2.CvtColor(gray, img, ColorConversionCodes.GRAY2BGR);

        if (path is not null)
        {
            foreach (var (px, py) in path)
                Cv2.Circle(img, new Point(px, py), 1, new Scalar(0, 255, 0), 1);
        }

        Cv2.Circle(img, new Point(goal.x, goal.y), 2, new Scalar(255, 0, 0), -1);
        Cv2.Circle(img, new Point(W / 2, H - 2), 2, new Scalar(0, 0, 255), -1);

        var scaled = new Mat();
        Cv2.Resize(img, scaled, new Size(W * displayScale, H * displayScale),
                   interpolation: InterpolationFlags.Nearest);
        img.Dispose();

        for (int i = 0; i < H; i++)
            Cv2.Line(scaled, new Point(0, i * displayScale),
                             new Point(W * displayScale, i * displayScale),
                             new Scalar(55, 55, 55), 1);
        for (int i = 0; i < W; i++)
            Cv2.Line(scaled, new Point(i * displayScale, 0),
                             new Point(i * displayScale, H * displayScale),
                             new Scalar(55, 55, 55), 1);

        return scaled;
    }
}