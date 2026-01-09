using OpenCvSharp;
using igvc_csharp.Utilities;

namespace igvc_csharp.Subsystems.Vision.Filters;

public sealed class HsvFilter : IFilter
{
    public enum OutputMode
    {
        WhiteForRange,     // default (lanes = white)
        WhiteForOutside    // inverted (ground = white)
    }

    private readonly Scalar _lower1;
    private readonly Scalar _upper1;
    private readonly Scalar? _lower2;
    private readonly Scalar? _upper2;
    private readonly bool _wrapsHue;
    private readonly OutputMode _outputMode;

    public HsvFilter(
        ColorUtilities.ColorRange range,
        OutputMode outputMode = OutputMode.WhiteForRange)
    {
        _outputMode = outputMode;

        var minH = ToCvHue(range.MinHue);
        var maxH = ToCvHue(range.MaxHue);

        var minS = ToCvByte(range.MinSaturation);
        var maxS = ToCvByte(range.MaxSaturation);

        var minV = ToCvByte(range.MinValue);
        var maxV = ToCvByte(range.MaxValue);

        _wrapsHue = minH > maxH;

        if (!_wrapsHue)
        {
            _lower1 = new Scalar(minH, minS, minV);
            _upper1 = new Scalar(maxH, maxS, maxV);
        }
        else
        {
            _lower1 = new Scalar(0, minS, minV);
            _upper1 = new Scalar(maxH, maxS, maxV);

            _lower2 = new Scalar(minH, minS, minV);
            _upper2 = new Scalar(179, maxS, maxV);
        }
    }

    public Mat Apply(Mat frame)
    {
        using var hsv = new Mat();
        using var mask = new Mat();

        Cv2.CvtColor(frame, hsv, ColorConversionCodes.BGR2HSV);
        Cv2.InRange(hsv, _lower1, _upper1, mask);

        if (_wrapsHue && _lower2.HasValue && _upper2.HasValue)
        {
            using var mask2 = new Mat();
            Cv2.InRange(hsv, _lower2.Value, _upper2.Value, mask2);
            Cv2.BitwiseOr(mask, mask2, mask);
        }

        if (_outputMode == OutputMode.WhiteForOutside)
        {
            Cv2.BitwiseNot(mask, mask);
        }

        Cv2.CvtColor(mask, frame, ColorConversionCodes.GRAY2BGR);
        return frame;
    }
    
    private static int ToCvHue(double hue)
        => (int)Math.Round(((hue % 360 + 360) % 360) / 2.0);

    private static int ToCvByte(double v)
        => (int)Math.Round(Math.Clamp(v, 0, 1) * 255);
}
