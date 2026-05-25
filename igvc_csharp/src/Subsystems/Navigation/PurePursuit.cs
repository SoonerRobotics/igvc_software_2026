namespace igvc_csharp.Subsystems.Navigation;

/// <summary>
/// Pure Pursuit path tracking algorithm.
/// Ported from https://github.com/xiaoxiae/PurePursuitAlgorithm
/// </summary>
public class PurePursuit
{
    private List<(float X, float Y)> _path = [];

    public void SetPoints(IEnumerable<(float X, float Y)> points)
    {
        _path = [.. points];
    }

    public void AddPoint(float x, float y) => _path.Add((x, y));

    public void Clear() => _path.Clear();

    public int Count => _path.Count;

    /// <summary>
    /// Returns the lookahead point on the path at radius <paramref name="r"/> from
    /// (<paramref name="x"/>, <paramref name="y"/>), or null if none exists.
    /// </summary>
    public (float X, float Y)? GetLookaheadPoint(float x, float y, float r)
    {
        (float X, float Y)? lookahead = null;

        for (int i = 0; i < _path.Count - 1; i++)
        {
            var segStart = _path[i];
            var segEnd = _path[i + 1];

            // Translate segment into robot-local coordinates
            var p1 = (X: segStart.X - x, Y: segStart.Y - y);
            var p2 = (X: segEnd.X - x, Y: segEnd.Y - y);

            float dx = p2.X - p1.X;
            float dy = p2.Y - p1.Y;
            float d = MathF.Sqrt(dx * dx + dy * dy);
            float D = p1.X * p2.Y - p2.X * p1.Y;

            float discriminant = r * r * d * d - D * D;
            if (discriminant < 0 || p1 == p2)
                continue;

            float sqrtDisc = MathF.Sqrt(discriminant);
            float sign = dy < 0 ? -1f : 1f;

            float x1 = (D * dy + sign * dx * sqrtDisc) / (d * d);
            float x2 = (D * dy - sign * dx * sqrtDisc) / (d * d);
            float y1 = (-D * dx + MathF.Abs(dy) * sqrtDisc) / (d * d);
            float y2 = (-D * dx - MathF.Abs(dy) * sqrtDisc) / (d * d);

            bool valid1 = (MathF.Min(p1.X, p2.X) < x1 && x1 < MathF.Max(p1.X, p2.X))
                       || (MathF.Min(p1.Y, p2.Y) < y1 && y1 < MathF.Max(p1.Y, p2.Y));

            bool valid2 = (MathF.Min(p1.X, p2.X) < x2 && x2 < MathF.Max(p1.X, p2.X))
                       || (MathF.Min(p1.Y, p2.Y) < y2 && y2 < MathF.Max(p1.Y, p2.Y));

            if (valid1 || valid2)
                lookahead = null;

            if (valid1)
                lookahead = (x1 + x, y1 + y);

            if (valid2)
            {
                bool firstFartherFromEnd = MathF.Abs(x1 - p2.X) > MathF.Abs(x2 - p2.X)
                                        || MathF.Abs(y1 - p2.Y) > MathF.Abs(y2 - p2.Y);
                if (lookahead is null || firstFartherFromEnd)
                    lookahead = (x2 + x, y2 + y);
            }
        }

        // Snap to end point when inside the lookahead radius
        if (_path.Count > 0)
        {
            var last = _path[^1];
            float distToEnd = MathF.Sqrt((last.X - x) * (last.X - x) + (last.Y - y) * (last.Y - y));
            if (distToEnd <= r)
                return (last.X, last.Y);
        }

        return lookahead;
    }
}