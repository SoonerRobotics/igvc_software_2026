using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision.Filters;

public class ZemlinInflationFilter(
    float maxRadius = 40f,
    float noGoPercent = 0.63f) : IFilter
{
    public Mat Apply(Mat frame)
    {
        // Ensure single-channel grayscale
        Mat gray;
        bool wasColor = frame.Channels() > 1;
        if (wasColor)
        {
            gray = new Mat();
            Cv2.CvtColor(frame, gray, ColorConversionCodes.BGR2GRAY);
        }
        else
        {
            gray = frame.Clone();
        }

        float noGoRadius = maxRadius * noGoPercent;
        float taperRange = maxRadius - noGoRadius;

        // Distance transform
        var inverted = new Mat();
        Cv2.BitwiseNot(gray, inverted);
        gray.Dispose();

        // CV_32F output; PRECISE mask gives the most accurate L2 distances.
        var distF = new Mat();
        Cv2.DistanceTransform(inverted, distF, DistanceTypes.L2, DistanceTransformMasks.Precise);
        inverted.Dispose();

        // Clamp distances to [noGoRadius, maxRadius] 
        Cv2.Max(distF, (double)noGoRadius, distF);
        Cv2.Min(distF, (double)maxRadius,  distF);

        // [noGoRadius, maxRadius] → [255, 0]
        var result = new Mat();
        distF.ConvertTo(
            result,
            MatType.CV_8U,
            alpha: -255.0 / taperRange,
            beta:   255.0 * maxRadius / taperRange);
        distF.Dispose();

        // Restore BGR channels
        if (!wasColor)
            return result;

        var bgrResult = new Mat();
        Cv2.CvtColor(result, bgrResult, ColorConversionCodes.GRAY2BGR);
        result.Dispose();
        return bgrResult;
    }
}
