using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision.Filters;

public class InflationFilter(int kernelWidth = 95, int kernelHeight = 95) : IFilter
{
    private readonly Mat _kernel = Cv2.GetStructuringElement(
        MorphShapes.Ellipse,
        new Size(
            kernelWidth % 2 == 0 ? kernelWidth + 1 : kernelWidth,
            kernelHeight % 2 == 0 ? kernelHeight + 1 : kernelHeight
        )
    );

    public Mat Apply(Mat frame)
    {
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

        // Downscale it to speed up the dilation
        // var downscaled = new Mat();
        // Cv2.Resize(gray, downscaled, new Size(gray.Width / 4, gray.Height / 4), 0, 0, InterpolationFlags.Area);
        // gray.Dispose();
        // gray = downscaled;

        // Single dilation pass — equivalent to Minkowski sum with the kernel shape.
        // Every white pixel expands outward by (kernelWidth/2, kernelHeight/2) pixels.
        Cv2.Dilate(gray, gray, _kernel);

        if (!wasColor)
            return gray;

        var result = new Mat();
        Cv2.CvtColor(gray, result, ColorConversionCodes.GRAY2BGR);
        gray.Dispose();
        return result;
    }
}