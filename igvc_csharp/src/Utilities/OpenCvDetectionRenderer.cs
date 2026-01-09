namespace igvc_csharp.Utilities;

using OpenCvSharp;

public static class OpenCvDetectionRenderer
{
    public static byte[] RenderDetections(
        byte[] jpeg,
        IReadOnlyList<igvc_csharp.Yolo.Detection> detections,
        int jpegQuality = 85)
    {
        using var mat = Cv2.ImDecode(jpeg, ImreadModes.Color);
        if (mat.Empty())
            return jpeg;

        foreach (var det in detections)
        {
            Cv2.Rectangle(mat, det.Bounding, Scalar.Red, 2);

            Cv2.PutText(
                mat,
                $"{det.Label} {det.Confidence:P1}",
                new Point(det.Bounding.X, Math.Max(0, det.Bounding.Y - 5)),
                HersheyFonts.HersheySimplex,
                0.5,
                Scalar.Yellow,
                1);
        }

        Cv2.ImEncode(
            ".jpg",
            mat,
            out var encoded,
            new ImageEncodingParam(ImwriteFlags.JpegQuality, jpegQuality));

        return encoded;
    }
}
