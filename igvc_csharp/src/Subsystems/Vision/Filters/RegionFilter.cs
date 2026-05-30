using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision.Filters;

/**
 * Mask a region to keep only pixels inside a polygon region (or zero pixels outside of it). Useful for removing
 * known issues like the robot, etc.
 */
public class RegionFilter : IFilter
{
    public enum RegionFilterMode
    {
        /// <summary>
        /// Keep only pixels inside the polygon; zero everything else.
        /// </summary>
        KeepInside,

        /// <summary>
        /// Zero out pixels inside the polygon; keep everything else.
        /// </summary>
        RemoveInside
    }

    private readonly Point[][] _contours;
    private readonly RegionFilterMode _mode;
    private readonly byte _fillValue;

    public RegionFilter(IEnumerable<Point> region, RegionFilterMode mode = RegionFilterMode.RemoveInside,
        byte fillValue = 255) : this([region], mode, fillValue)
    {
    }

    public RegionFilter(IEnumerable<IEnumerable<Point>> regions, RegionFilterMode mode = RegionFilterMode.RemoveInside,
        byte fillValue = 255)
    {
        _contours = regions.Select(r => r.ToArray()).ToArray();
        _mode = mode;
        _fillValue = fillValue;
    }

public Mat Apply(Mat frame)
{
    using var mask = BuildMask(frame.Size());
    using var invertedMask = new Mat();
    var result = frame.Clone();
    var fillScalar = new Scalar(_fillValue, _fillValue, _fillValue);

    if (_mode == RegionFilterMode.KeepInside)
    {
        Cv2.BitwiseNot(mask, invertedMask);
        result.SetTo(fillScalar, invertedMask);
    }
    else
    {
        result.SetTo(fillScalar, mask);
    }

    return result;
}

    private Mat BuildMask(Size size)
    {
        var mask = Mat.Zeros(size.Height, size.Width, MatType.CV_8UC1);
        Cv2.FillPoly(mask, _contours, Scalar.White);
        return mask;
    }
}