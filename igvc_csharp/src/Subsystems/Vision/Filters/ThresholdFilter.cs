using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision.Filters;

public class ThresholdFilter(byte threshold = 250) : IFilter
{
    public Mat Apply(Mat frame)
    {
        Mat gray;
        if (frame.Channels() == 1)
        {
            gray = frame.Clone();
        }
        else
        {
            gray = new Mat();
            Cv2.CvtColor(frame, gray, ColorConversionCodes.BGR2GRAY);
        }

        Cv2.Threshold(gray, gray, threshold, 255, ThresholdTypes.Binary);

        if (frame.Channels() == 1)
        {
            return gray;
        }

        var result = new Mat();
        Cv2.CvtColor(gray, result, ColorConversionCodes.GRAY2BGR);
        gray.Dispose();
        return result;
    }
}