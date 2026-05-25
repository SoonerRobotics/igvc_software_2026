using OpenCvSharp;

namespace igvc_csharp.Subsystems.Navigation;

/// <summary>
/// Grid dimensions and FOV config that drive coordinate conversions.
/// Adjust these to match your camera/map setup.
/// </summary>
public record AStarConfig
{
    /// <summary>Grid width and height in cells.</summary>
    public int GridWidth { get; init; } = 80;
    public int GridHeight { get; init; } = 80;

    /// <summary>
    /// Camera field of view used when converting grid cells to robot-local
    /// metric coordinates for the path follower.
    /// </summary>
    public float HorizontalFov { get; init; } = 3.4f;
    public float VerticalFov { get; init; } = 2.75f;

    /// <summary>
    /// Weights for the frontier scoring heuristic.
    /// depthWeight:    reward for exploring further from the robot
    /// forwardWeight:  reward for cells closer to the top of the map (forward)
    /// </summary>
    public float DepthWeight { get; init; } = 2.2f;
    public float ForwardWeight { get; init; } = 1.3f;

    /// <summary>Maximum BFS depth when searching for the best goal cell.</summary>
    public int MaxFrontierDepth { get; init; } = 50;

    /// <summary>Obstacle threshold: cells with cost >= this are impassable.</summary>
    public byte ObstacleThreshold { get; init; } = 50;
}

/// <summary>
/// Stateless A* planner operating on a flat occupancy grid.
/// The grid is row-major: index = x + width * y.
/// Origin (robot position) is fixed at (gridWidth/2, gridHeight-2).
/// </summary>
public static class AStarPlanner
{
    // All 8 neighbors with their Euclidean movement cost
    private static readonly (int dx, int dy, float cost)[] SearchDirs =
        Enumerable.Range(-1, 3)
            .SelectMany(dx => Enumerable.Range(-1, 3)
                .Where(dy => !(dx == 0 && dy == 0))
                .Select(dy => (dx, dy, MathF.Sqrt(dx * dx + dy * dy))))
            .ToArray();

    /// <param name="costMap">
    ///   Flat occupancy grid, row-major, values 0-255.
    ///   Values >= config.ObstacleThreshold are treated as obstacles.
    /// </param>
    /// <param name="config">Planner configuration.</param>
    /// <returns>
    ///   An ordered list of (x,y) grid cells from robot position to goal,
    ///   or null if no path exists.
    /// </returns>
    public static List<(int x, int y)>? FindPath(byte[] costMap, AStarConfig config)
    {
        int W = config.GridWidth;
        int H = config.GridHeight;

        var robotPos = (x: W / 2, y: H - 2);
        var goalPos = FindBestGoal(costMap, robotPos, config);

        return AStar(robotPos, goalPos, costMap, config);
    }

    // ── Frontier BFS to find the best reachable goal cell ──────────────────

    private static (int x, int y) FindBestGoal(
        byte[] costMap,
        (int x, int y) robotPos,
        AStarConfig config)
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

                if (score > bestCost)
                {
                    bestCost = score;
                    bestPos = (x, y);
                }

                explored.Add(x + W * y);

                // 4-connected expansion (matches Python original)
                TryAdd(x, y - 1, costMap, W, H, explored, frontier, config);
                TryAdd(x + 1, y, costMap, W, H, explored, frontier, config);
                TryAdd(x - 1, y, costMap, W, H, explored, frontier, config);
                // Python omits downward expansion; preserved intentionally
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

    // ── A* ──────────────────────────────────────────────────────────────────

    private static List<(int x, int y)>? AStar(
        (int x, int y) start,
        (int x, int y) goal,
        byte[] costMap,
        AStarConfig config)
    {
        int W = config.GridWidth;
        int H = config.GridHeight;

        // closed set replaces the O(n) open_set.remove() in the Python version
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

        // Heuristic: Euclidean distance to goal
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

        return null; // no path found
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

    // ── Coordinate conversion ────────────────────────────────────────────────

    /// <summary>
    /// Converts a grid cell to robot-local metric coordinates (x = forward, y = left).
    /// Mirrors path_to_global() in the Python node, with the identity rotation kept
    /// but made explicit (robot heading = 0 in local frame by definition).
    /// </summary>
    public static (float x, float y) GridToLocal((int gx, int gy) cell, AStarConfig config)
    {
        float x = (config.GridHeight - cell.gy) * config.VerticalFov / config.GridHeight;
        float y = (config.GridWidth / 2f - cell.gx) * config.HorizontalFov / config.GridWidth;
        // rotation by 0 is identity — preserved for future heading correction
        return (x, y);
    }

    // ── Debug image ──────────────────────────────────────────────────────────

    /// <summary>
    /// Renders the occupancy grid with the planned path overlaid.
    /// Returns a BGR Mat scaled up for visibility.
    /// </summary>
    public static Mat BuildDebugImage(
        byte[] costMap,
        List<(int x, int y)>? path,
        (int x, int y) goal,
        AStarConfig config,
        int displayScale = 4)
    {
        int W = config.GridWidth;
        int H = config.GridHeight;

        // Cost map → grayscale → BGR
        using var gray = new Mat(H, W, MatType.CV_8UC1);
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                gray.At<byte>(y, x) = costMap[x + W * y];

        var img = new Mat();
        Cv2.CvtColor(gray, img, ColorConversionCodes.GRAY2BGR);

        // Draw path in green
        if (path is not null)
        {
            foreach (var (px, py) in path)
                Cv2.Circle(img, new Point(px, py), 1, new Scalar(0, 255, 0), 1);
        }

        // Draw goal in blue
        Cv2.Circle(img, new Point(goal.x, goal.y), 2, new Scalar(255, 0, 0), -1);

        // Draw robot origin in red
        var robot = new Point(W / 2, H - 2);
        Cv2.Circle(img, robot, 2, new Scalar(0, 0, 255), -1);

        // Scale up for readability
        var scaled = new Mat();
        Cv2.Resize(img, scaled, new Size(W * displayScale, H * displayScale),
                   interpolation: InterpolationFlags.Nearest);
        img.Dispose();

        // Grid lines at original cell boundaries
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