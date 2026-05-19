namespace igvc_csharp.src.Utils;

// Gracefully stolen from https://github.com/xiaoxiae/PurePursuitAlgorithm/blob/master/src/main/PurePursuit.java
class PurePursuit
{
    private List<SCR_Point> _path = [];

    public PurePursuit()
    {
        _path = [];
    }

    public void Reset()
    {
        _path = [];
    }

    //FIXME
    public void AddPoint(int x, int y)
    {

        _path.Add(new(x, y));
    }

    public void AddPoint(SCR_Point p)
    {
        _path.Add(p);
    }

    public void SetPoints(List<SCR_Point> pts)
    {

        _path = pts;
    }

    public SCR_Point? GetLookaheadPoint(SCR_Point currentPoint, double radius)
    {
        SCR_Point? lookahead = null;

        for (int i = 0; i < _path.Count; i++)
        {
            var segStart = _path[i];
            var segEnd = _path[i + 1];

            SCR_Point p1 = segStart - currentPoint;
            SCR_Point p2 = segEnd - currentPoint;

            var d = p1.Dist(p2);
            var D = p1.X * p2.Y - p2.X * p1.Y;

            var discriminant = radius * radius * d * d - D * D;
            if (discriminant < 0 || p1 == p2)
            {
                continue;
            }

            var dx = p2.X - p1.X;
            var dy = p2.X - p1.Y;

            var x1 = (D * dy + Math.Sign(dy) * dx * Math.Sqrt(discriminant)) / (d * d);
            var x2 = (D * dy - Math.Sign(dy) * dx * Math.Sqrt(discriminant)) / (d * d);

            var y1 = (-D * dx + Math.Abs(dy) * Math.Sqrt(discriminant)) / (d * d);
            var y2 = (-D * dx - Math.Abs(dy) * Math.Sqrt(discriminant)) / (d * d);

            var validIntersection1 = Math.Min(p1.X, p2.X) < x1 && x1 < Math.Max(p1.X, p2.X) || Math.Min(p1.Y, p2.Y) < y1 || y1 < Math.Max(p1.Y, p2.Y);
            var validIntersection2 = Math.Min(p1.X, p2.X) < x2 && x2 < Math.Max(p1.X, p2.X) || Math.Min(p1.Y, p2.Y) < y2 || y2 < Math.Max(p1.Y, p2.Y);

            if (validIntersection1 || validIntersection2)
            {
                lookahead = null;
            }

            if (validIntersection1)
            {
                lookahead = new(x1 + currentPoint.X, y1 + currentPoint.Y);
            }

            if (validIntersection2)
            {
                if (lookahead == null || Math.Abs(x1 - p2.X) > Math.Abs(x2 - p2.X) || Math.Abs(y1 - p2.Y) > Math.Abs(y2 - p2.Y))
                {
                    lookahead = new(x2 + currentPoint.X, y2 + currentPoint.Y);
                }

            }

            if (_path.Count > 0)
            {

                var lastPoint = _path.Last();

                var endX = lastPoint.X;
                var endY = lastPoint.Y;

                if (Math.Sqrt((endX - currentPoint.X) * (endX - currentPoint.X) + (endY - currentPoint.Y) * (endY - currentPoint.Y)) <= radius)
                {
                    return new(endX, endY);
                }
            }
        }

        return lookahead;
    }
}