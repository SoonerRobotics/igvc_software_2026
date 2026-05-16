using OpenCvSharp;

namespace igvc_csharp.Utils;

public static class OpenCvDetectionRenderer
{
    public static byte[] RenderDetections(
        byte[] jpeg,
        IReadOnlyList<DetectionWithDepth> detections,
        int jpegQuality = 85)
    {
        using var mat = Cv2.ImDecode(jpeg, ImreadModes.Color);
        if (mat.Empty()) return jpeg;

        foreach (var d in detections)
            DrawDetection(mat, d);

        Cv2.ImEncode(
            ".jpg",
            mat,
            out var encoded,
            new ImageEncodingParam(ImwriteFlags.JpegQuality, jpegQuality));

        return encoded;
    }

    public static byte[] RenderDetections(
        byte[] jpeg,
        IReadOnlyList<igvc_csharp.Yolo.Detection> detections,
        int jpegQuality = 85)
    {
        return RenderDetections(
            jpeg,
            detections.Select(d => new DetectionWithDepth(d, float.NaN)).ToList(),
            jpegQuality
        );
    }

    private static void DrawDetection(Mat mat, DetectionWithDepth d)
    {
        var box     = d.Detection.Bounding;
        var color   = DepthToColor(d.DistanceMeters);
        var distStr = float.IsNaN(d.DistanceMeters) ? "no depth" : $"{d.DistanceMeters:F2}m";
        var label   = $"{d.Detection.Label}  {distStr}";

        int cornerLen = Math.Min(box.Width, box.Height) / 5;
        DrawCornerAccents(mat, box, color, cornerLen, thickness: 2);

        var lineStart = new Point(box.X + box.Width / 2, box.Y);
        var lineElbow = new Point(box.X + box.Width / 2, box.Y - 20);
        var lineEnd   = new Point(lineElbow.X + 40,      lineElbow.Y);

        Cv2.Line(mat, lineStart, lineElbow, color, 1, LineTypes.AntiAlias);
        Cv2.Line(mat, lineElbow, lineEnd,   color, 1, LineTypes.AntiAlias);

        const double scale     = 0.45;
        const int    textThick = 1;
        var textSize = Cv2.GetTextSize(label, HersheyFonts.HersheySimplex, scale, textThick, out _);

        const int padX = 6, padY = 4;
        var pillTl = new Point(lineEnd.X, lineEnd.Y - textSize.Height - padY);
        var pillBr = new Point(lineEnd.X + textSize.Width + padX * 2, lineEnd.Y + padY);

        Cv2.Rectangle(mat,
            new Rect(pillTl.X, pillTl.Y, pillBr.X - pillTl.X, pillBr.Y - pillTl.Y),
            color, thickness: -1);

        Cv2.PutText(mat, label,
            new Point(pillTl.X + padX, lineEnd.Y),
            HersheyFonts.HersheySimplex, scale,
            Scalar.White, textThick, LineTypes.AntiAlias);
    }

    private static void DrawCornerAccents(Mat mat, Rect box, Scalar color, int len, int thickness)
    {
        var tl = new Point(box.X,             box.Y);
        var tr = new Point(box.X + box.Width, box.Y);
        var bl = new Point(box.X,             box.Y + box.Height);
        var br = new Point(box.X + box.Width, box.Y + box.Height);

        Cv2.Line(mat, tl, tl + new Point(len,  0),   color, thickness, LineTypes.AntiAlias);
        Cv2.Line(mat, tl, tl + new Point(0,    len),  color, thickness, LineTypes.AntiAlias);
        Cv2.Line(mat, tr, tr + new Point(-len, 0),   color, thickness, LineTypes.AntiAlias);
        Cv2.Line(mat, tr, tr + new Point(0,    len),  color, thickness, LineTypes.AntiAlias);
        Cv2.Line(mat, bl, bl + new Point(len,  0),   color, thickness, LineTypes.AntiAlias);
        Cv2.Line(mat, bl, bl + new Point(0,    -len), color, thickness, LineTypes.AntiAlias);
        Cv2.Line(mat, br, br + new Point(-len, 0),   color, thickness, LineTypes.AntiAlias);
        Cv2.Line(mat, br, br + new Point(0,    -len), color, thickness, LineTypes.AntiAlias);
    }

    private static Scalar DepthToColor(float meters)
    {
        if (float.IsNaN(meters)) return Scalar.Gray;
        float t = Math.Clamp((meters - 0.2f) / (10f - 0.2f), 0f, 1f);
        return new Scalar(0, (int)(200 * (1f - t)), (int)(200 * t));
    }
}