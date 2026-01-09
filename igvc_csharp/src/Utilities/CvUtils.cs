using Messages;
using OpenCvSharp;

namespace igvc_csharp.Utilities;

public static class CvUtils
{
    public static Mat AsMat(ImageFrame frame)
    {
        var byts = frame.GetImageDataArray();
        return Cv2.ImDecode(byts, ImreadModes.Color);
    }

    public static byte[] FromMat(Mat mat)
    {
        Cv2.ImEncode(".jpg", mat, out var buf);
        return buf;
    }

    public static Point2f[] GetCheckerboardCorners(
        Mat image,
        int rows,
        int cols)
    {
        if (image.Empty())
        {
            throw new ArgumentException("Image is empty", nameof(image));
        }

        if (rows <= 0 || cols <= 0)
        {
            throw new ArgumentOutOfRangeException("Rows and columns must be positive");
        }

        using var gray = new Mat();

        if (image.Channels() == 1)
        {
            image.CopyTo(gray);
        }
        else
        {
            Cv2.CvtColor(image, gray, ColorConversionCodes.BGR2GRAY);
        }

        var patternSize = new Size(cols, rows);
        var found = Cv2.FindChessboardCorners(
            gray,
            patternSize,
            out var corners
        );

        if (!found)
        {
            throw new InvalidOperationException("Checkerboard pattern not found");
        }

        // A little spice to clean things up
        Cv2.CornerSubPix(
            gray,
            corners,
            new Size(11, 11),
            new Size(-1, -1),
            new TermCriteria(
                CriteriaTypes.Eps | CriteriaTypes.MaxIter,
                30,
                0.001)
        );

        return corners;
    }
}