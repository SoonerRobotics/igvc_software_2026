using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision.Filters;

public sealed class WinningFilter : IFilter
{
    // HSV thresholds (mirror Python defaults)
    private readonly Scalar _lower = new Scalar(0, 0, 0);
    private readonly Scalar _upper = new Scalar(255, 95, 210);

    // Blur
    private readonly Size _blurSize = new Size(5, 5);
    private const int BlurIterations = 3;

    // Region of disinterest
    private const int RoiOffset = 130;

    // Occupancy map output resolution
    private const int MapRes = 80;

    public Mat Apply(Mat frame)
    {
        Mat img = frame.Clone();

        // 1. Blur
        for (int i = 0; i < BlurIterations; i++)
            Cv2.Blur(img, img, _blurSize);

        // 2. HSV threshold → mask
        Mat hsv = new Mat();
        Cv2.CvtColor(img, hsv, ColorConversionCodes.BGR2HSV);

        Mat mask = new Mat();
        Cv2.InRange(hsv, _lower, _upper, mask);

        // Invert: white = traversable (mirrors Python's 255 - mask)
        Cv2.BitwiseNot(mask, mask);

        // 3. Region of disinterest (triangle cutout)
        int h = frame.Rows;
        int w = frame.Cols;
        Point[] roiVertices =
        [
            new Point(0, h),
            new Point(w / 2, h / 2 + RoiOffset),
            new Point(w, h)
        ];

        Mat roiMask = Mat.Ones(mask.Size(), MatType.CV_8UC1) * 255;
        Cv2.FillPoly(roiMask, [roiVertices], Scalar.Black);
        Cv2.BitwiseAnd(mask, roiMask, mask);

        // Hard threshold — only keep near-white pixels
        mask.SetTo(Scalar.Black, mask.LessThan(250));

        // 4. Perspective flatten (bird's-eye view)
        mask = FlattenImage(mask);

        // 5. Resize to occupancy map and scale to [0, 100]
        Mat mapMat = new Mat();
        Cv2.Resize(mask, mapMat, new Size(MapRes, MapRes), interpolation: InterpolationFlags.Linear);
        mapMat.ConvertTo(mapMat, MatType.CV_32FC1, 1.0 / 2.0); // divide by 2 → [0, 127]

        img.Dispose();
        hsv.Dispose();
        mask.Dispose();
        roiMask.Dispose();

        return mapMat; // float Mat, values 0–127 (occupancy)
    }

    private static Mat FlattenImage(Mat img)
    {
        int h = img.Rows;  // 480
        int w = img.Cols;  // 640

        // Source points — same trapezoid as the Python code
        Point2f[] src =
        [
            new Point2f(w * 0.26f,        h),   // top-left  (near-bottom in image)
            new Point2f(w - w * 0.26f,    h),   // top-right
            new Point2f(0,                0),   // bottom-left
            new Point2f(w,                0),   // bottom-right
        ];

        // Destination points — full rectangle
        Point2f[] dst =
        [
            new Point2f(0,   h),
            new Point2f(w,   h),
            new Point2f(0,   0),
            new Point2f(w,   0),
        ];

        // Python calls getPerspectiveTransform(dest, src) then warpPerspective,
        // which is equivalent to getTransform(src, dst) + warp here.
        Mat matrix = Cv2.GetPerspectiveTransform(src, dst);
        Mat output = new Mat();
        Cv2.WarpPerspective(img, output, matrix, new Size(w, h));
        matrix.Dispose();
        return output;
    }
}