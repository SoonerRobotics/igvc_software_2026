using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision.Filters;

public class InflationFilter : IFilter
{
    private struct InflationCell(int deltaX, int deltaY, double radiusInCells)
    {
        public readonly int DeltaX = deltaX;
        public readonly int DeltaY = deltaY;
        public readonly double RadiusInCells = radiusInCells;
    }

    private readonly List<InflationCell> _inflationCells = [];
    private int _hardNoGoRadiusCell = 0;
    private int _maxRadiusCells = 0;

    private readonly float _inflationRadiusMeters;
    private readonly float _hardNoGoFraction;
    private readonly float _horizontalFov;
    private readonly byte _obstacleThreshold;
    private readonly byte _hardNoGoCost;
    
    /**
     * inflationRadiusMeters: Maximum inflation radius in meters (e.g. how big are obstacles expanded)<br/>
     * hardNoGoFraction: The fraction of inflationRadiusMeters that is treated as a hard no-go zone<br/>
     * horizontalFovMeters: The horizontal fov of the camera<br/>
     * obstacleThreshold: The threshold required to treat something as an obstacle<br/>
     * hardNoGoCost: What cost are no-go cells assigned
     */
    public InflationFilter(
        float inflationRadiusMeters = 1.0f,
        float hardNoGoFraction = 0.7f,
        float horizontalFovMeters = 3.4f,
        byte obstacleThreshold = 1,
        byte hardNoGoCost = 100
    )
    {
        _inflationRadiusMeters = inflationRadiusMeters;
        _hardNoGoFraction = hardNoGoFraction;
        _horizontalFov = horizontalFovMeters;
        _obstacleThreshold = obstacleThreshold;
        _hardNoGoCost = hardNoGoCost;
    }

    /**
     * Computes the circles array if it is not initialized
     */
    private void ComputeCircles(int width)
    {
        if (_inflationCells.Count != 0)
        {
            return;
        }

        var cellsPerMeter = width / _horizontalFov;
        _maxRadiusCells = (int)(_inflationRadiusMeters * cellsPerMeter);
        _hardNoGoRadiusCell = (int)(_inflationRadiusMeters * _hardNoGoFraction * cellsPerMeter);
        _inflationCells.Add(new InflationCell(0, 0, 0));

        for (var dx = -_maxRadiusCells; dx <= _maxRadiusCells; dx++)
        {
            for (var dy = -_maxRadiusCells; dy <= _maxRadiusCells; dy++)
            {
                var radius  = Math.Sqrt(dx * dx + dy * dy);
                if (radius > 0 && radius < _maxRadiusCells)
                {
                    _inflationCells.Add(new InflationCell(dx, dy, radius));
                }
            }
        }
    }

    /**
     * Maps a distance from an obstacle to a cost value (0 - 100)
     */
    private byte ComputeCost(double radiusInCells)
    {
        if (radiusInCells <= _hardNoGoRadiusCell)
        {
            return _hardNoGoCost;
        }
        
        var t = (radiusInCells - _hardNoGoRadiusCell) / (_maxRadiusCells - _hardNoGoRadiusCell);
        var cost = _hardNoGoCost * (1.0 - t);
        return (byte)Math.Max(1, Math.Round(cost));
    }
    
    public Mat Apply(Mat frame)
    {
        ComputeCircles(frame.Width);

        var expanded = (Mat)Mat.Zeros(frame.Width, frame.Height, MatType.CV_8UC1);
        var rawIndexer = frame.GetGenericIndexer<byte>();
        var expandedIndexer = expanded.GetGenericIndexer<byte>();
        for (var y = 0; y < frame.Height; y++)
        {
            for (var x = 0; x < frame.Width; x++)
            {
                // Skip empty cells
                if (rawIndexer[y, x] < _obstacleThreshold)
                {
                    continue;
                }

                foreach (var cell in _inflationCells)
                {
                    var nx = x * cell.DeltaX;
                    var ny = y + cell.DeltaY;
                    if (nx < 0 || nx >= frame.Width || ny < 0 || ny >= frame.Height)
                    {
                        continue;
                    }

                    var newCost = ComputeCost(cell.RadiusInCells);
                    if (expandedIndexer[ny, nx] < newCost)
                    {
                        expandedIndexer[ny, nx] = newCost;
                    }
                }
            }
        }
        
        return expanded;
    }
}