using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision.Filters;

public class TopDownFilter(Point2f[] sourcePoints, Point2f[] destinationPoints, Size outputSize)
    : IFilter
{
    public TopDownFilter(Point2f[] sourcePoints, Size outputSize) : this(sourcePoints,
        GetFullRectDestination(outputSize), outputSize)
    {
    }

    public Mat Apply(Mat frame)
    {
        var result = new Mat();
        Cv2.WarpPerspective(frame, result, TransformMatrix, outputSize, InterpolationFlags.Linear,
            BorderTypes.Constant, Scalar.Black);
        return result;
    }

    private Mat TransformMatrix { get; } = Cv2.GetPerspectiveTransform(sourcePoints, destinationPoints);

    private static Point2f[] GetFullRectDestination(Size outputSize) =>
    [
        new(0, 0), // top-left
        new(outputSize.Width, 0), // top-right
        new(outputSize.Width, outputSize.Height), // bottom-right
        new(0, outputSize.Height) // bottom-left
    ];

    public Mat Draw(Mat frame)
    {
        var result = frame.Clone();
        Cv2.Line(result, sourcePoints[0].ToPoint(), sourcePoints[1].ToPoint(), Scalar.Red, 2);
        Cv2.Line(result, sourcePoints[1].ToPoint(), sourcePoints[2].ToPoint(), Scalar.Red, 2);
        Cv2.Line(result, sourcePoints[2].ToPoint(), sourcePoints[3].ToPoint(), Scalar.Red, 2);
        Cv2.Line(result, sourcePoints[3].ToPoint(), sourcePoints[0].ToPoint(), Scalar.Red, 2);
        return result;
    }
}