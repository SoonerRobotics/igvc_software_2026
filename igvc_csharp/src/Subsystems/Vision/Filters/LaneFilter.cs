using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision.Filters;

public class LaneDetectionFilter : IFilter
{
    public Mat Apply(Mat input)
    {
        using var hls = new Mat();
        using var whiteMask = new Mat();
        using var yellowMask = new Mat();

        Cv2.CvtColor(input, hls, ColorConversionCodes.BGR2HLS);

        Cv2.InRange(hls, new Scalar(0, 210, 0), new Scalar(255, 255, 30), whiteMask);
        Cv2.InRange(hls, new Scalar(15, 30, 100), new Scalar(35, 204, 255), yellowMask);

        using var kernel = Cv2.GetStructuringElement(MorphShapes.Rect, new Size(2, 2));

        using var tmpW = new Mat();
        using var cleanedWhite = new Mat();
        Cv2.MorphologyEx(whiteMask, tmpW, MorphTypes.Open, kernel);
        Cv2.MorphologyEx(tmpW, cleanedWhite, MorphTypes.Close, kernel);

        using var tmpY = new Mat();
        using var passV = new Mat();
        using var cleanedYellow = new Mat();
        using var kernelV = Cv2.GetStructuringElement(MorphShapes.Rect, new Size(2, 40));
        using var kernelH = Cv2.GetStructuringElement(MorphShapes.Rect, new Size(10, 2));

        Cv2.MorphologyEx(yellowMask, tmpY, MorphTypes.Open, kernel);
        Cv2.MorphologyEx(tmpY, passV, MorphTypes.Close, kernelV);
        Cv2.MorphologyEx(passV, cleanedYellow, MorphTypes.Close, kernelH);

        var output = new Mat(input.Size(), MatType.CV_8UC3, Scalar.Black);
        output.SetTo(new Scalar(255, 255, 255), cleanedWhite);
        output.SetTo(new Scalar(0, 255, 255), cleanedYellow);
        return output;
    }
}