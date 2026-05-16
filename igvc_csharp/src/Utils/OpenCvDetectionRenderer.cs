using OpenCvSharp;

namespace igvc_csharp.Utils;

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
            Cv2.Rectangle(mat, det.Bounding, Scalar.Red, 4);

            Cv2.PutText(
                mat,
                $"{det.Label} {det.Confidence:P1}",
                new Point(det.Bounding.X, det.Bounding.Y),
                HersheyFonts.HersheyPlain,
                0.3,
                Scalar.WhiteSmoke,
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
