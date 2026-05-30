using System.Buffers;
using System.Runtime.InteropServices;
using igvc_csharp.src.subsystems.selfdrive;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;
using Tesseract;
using ZstdSharp;

namespace igvc_csharp.Utils;

public class CvUtils
{
    private static ILogger Logger = Logging.From<CvUtils>();

    public static Mat AsMat(ImageFrame frame)
    {
        var bytes = frame.GetImageDataArray();
        return Cv2.ImDecode(bytes, ImreadModes.Color);
    }

    public static Mat AsMat(byte[] imageData)
    {
        return Cv2.ImDecode(imageData, ImreadModes.Color);
    }

    public static Mat AsDepthMat(ZedFrame frame) => ProcessDepthMessage(frame);

    public static Mat ProcessDepthMessage(ZedFrame frame)
    {
        var w = frame.Width;
        var h = frame.Height;
        var compressed = frame.GetDataArray();
        var floatCount = w * h;
        var byteCount = floatCount * sizeof(float);

        var floatBytes = ArrayPool<byte>.Shared.Rent(byteCount);
        try
        {
            using var decompressor = new Decompressor();
            var decompressed = decompressor.Unwrap(compressed, floatBytes);

            if (decompressed != byteCount)
            {
                throw new InvalidDataException($"Decompressed size {decompressed} != expected {byteCount}");
            }

            var floats = MemoryMarshal.Cast<byte, float>(floatBytes.AsSpan(0, byteCount));
            var mat = new Mat(h, w, MatType.CV_32FC1);
            unsafe
            {
                fixed (float* src = floats)
                    Buffer.MemoryCopy(
                        src,
                        mat.DataPointer,
                        (long)byteCount,
                        (long)byteCount
                    );
            }

            return mat;
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(floatBytes);
        }
    }

    public static float SampleDepth(Mat depthMat, OpenCvSharp.Rect bounding)
    {
        int cx = bounding.X + bounding.Width / 2;
        int cy = bounding.Y + bounding.Height / 2;

        // Clamp center to mat bounds first
        cx = Math.Clamp(cx, 0, depthMat.Width - 1);
        cy = Math.Clamp(cy, 0, depthMat.Height - 1);

        const int sampleRadius = 8; // slightly larger window for better coverage
        int x0 = Math.Max(0, cx - sampleRadius);
        int y0 = Math.Max(0, cy - sampleRadius);
        int x1 = Math.Min(depthMat.Width - 1, cx + sampleRadius);
        int y1 = Math.Min(depthMat.Height - 1, cy + sampleRadius);

        var samples = new List<float>(capacity: (x1 - x0 + 1) * (y1 - y0 + 1));

        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
            {
                float d = depthMat.At<float>(y, x);
                if (d >= 0.2f && d <= 20f)
                    samples.Add(d);
            }

        if (samples.Count == 0) return float.NaN;

        // Median is much more robust than mean for sparse/noisy depth data
        samples.Sort();
        return samples[samples.Count / 2];
    }

    public static Mat AsColorizedMat(ZedFrame frame, float minMeters = 0.2f, float maxMeters = 20f)
    {
        var mat = ProcessDepthMessage(frame);

        var normalized = new Mat();
        mat.ConvertTo(
            normalized,
            MatType.CV_8UC1,
            255.0 / (maxMeters - minMeters),
            -minMeters * 255.0 / (maxMeters - minMeters)
        );

        var colorized = new Mat();
        Cv2.ApplyColorMap(normalized, colorized, ColormapTypes.Jet);

        var mask = new Mat();
        Cv2.Compare(mat, new Scalar(0.001f), mask, CmpTypes.LT);
        colorized.SetTo(new Scalar(0, 0, 0), mask);

        normalized.Dispose();
        mask.Dispose();
        mat.Dispose();

        return colorized;
    }

    public static byte[] FromMat(Mat mat)
    {
        Cv2.ImEncode(".jpg", mat, out var buf);
        return buf;
    }

    public static Mat CloneMat(ImageFrame frame)
    {
        var mat = AsMat(frame);
        return mat.Clone();
    }

    public static Mat CloneMat(Mat mat) => mat.Clone();

    public static MessageWrapper BuildWrapper(uint width, uint height, string id, byte[] frame)
    {
        var newFrame = MessageConstructor.CreateImageFrame(width, height, id, frame);
        return MessageWrapper.From(MessageType.ImageFrame, newFrame.ByteBuffer.ToFullArray());
    }

    // Histogram stuff

    /// <summary>
    /// Extracts the HSV color range from a mat
    /// Takes in a point and a radius to sample around, and returns the min/max HSV values in that area within the 10% lower and upper percentiles to filter out outliers
    /// </summary>
    public static ColorUtils.ColorRange? ExtractHsvRange(Mat mat, Point center, int radius)
    {
        int x0 = Math.Max(0, center.X - radius);
        int y0 = Math.Max(0, center.Y - radius);
        int x1 = Math.Min(mat.Width - 1, center.X + radius);
        int y1 = Math.Min(mat.Height - 1, center.Y + radius);

        var hsvValues = new List<Vec3b>((x1 - x0 + 1) * (y1 - y0 + 1));
        for (int y = y0; y <= y1; y++)
        {
            for (int x = x0; x <= x1; x++)
            {
                hsvValues.Add(mat.At<Vec3b>(y, x));
            }
        }

        if (hsvValues.Count == 0)
        {
            return null;
        }

        hsvValues.Sort((a, b) =>
        {
            int ha = a.Item0 * 256 * 256 + a.Item1 * 256 + a.Item2;
            int hb = b.Item0 * 256 * 256 + b.Item1 * 256 + b.Item2;
            return ha.CompareTo(hb);
        });

        int lowerIndex = (int)(hsvValues.Count * 0.1);
        int upperIndex = (int)(hsvValues.Count * 0.9);

        var lower = hsvValues[lowerIndex];
        var upper = hsvValues[upperIndex];

        return ColorUtils.ColorRange.From(
            ColorUtils.Color.FromHsv(lower.Item0, lower.Item1, lower.Item2),
            ColorUtils.Color.FromHsv(upper.Item0, upper.Item1, upper.Item2)
        );
    }

    public static void DrawHsvTargetCircle(Mat mat, Point center, int radius)
    {
        Cv2.Circle(mat, center, radius, new Scalar(0, 255, 255), 2);
    }

    public static Point GetCenterPoint(Mat mat)
    {
        return new Point(mat.Width / 2, mat.Height / 2);
    }

    private static readonly Lazy<TesseractEngine> _engine = new(() =>
    {
        var engine = new TesseractEngine(
            FileUtils.GetFileRelativeToRoot("resources/eng.traineddata"),
            "eng",
            EngineMode.LstmOnly  // faster than Default for simple text
        );
        engine.SetVariable("tessedit_char_whitelist", "STOP");
        engine.SetVariable("tessedit_pageseg_mode", "8"); // PSM_WORD — single word mode
        return engine;
    });

    // Reusable Mats to avoid per-call allocation
    [ThreadStatic] private static Mat? _gray;
    [ThreadStatic] private static Mat? _upscaled;
    [ThreadStatic] private static Mat? _thresh;

    public static bool IsValidStopsign(Mat mat)
    {
        _gray ??= new Mat();
        _upscaled ??= new Mat();
        _thresh ??= new Mat();

        Cv2.CvtColor(mat, _gray, ColorConversionCodes.RGB2GRAY);
        Cv2.Resize(_gray, _upscaled, new Size(), 2, 2, InterpolationFlags.Cubic);
        Cv2.Threshold(_upscaled, _thresh, 0, 255, ThresholdTypes.Otsu | ThresholdTypes.BinaryInv);

        using var pix = Pix.LoadFromMemory(_thresh.ToBytes(".png"));
        using var page = _engine.Value.Process(pix, PageSegMode.SingleWord);

        var text = page.GetText().Trim();
        Logger.LogInformation("OCR Result: '{Text}'", text);
        return text.Equals("STOP", StringComparison.OrdinalIgnoreCase);
    }

    public static SelfdriveLane ExtractLane(Mat left, Mat right)
    {
        using var leftYellow = new Mat();
        using var rightYellow = new Mat();
        Cv2.InRange(left, new Scalar(0, 254, 254), new Scalar(1, 255, 255), leftYellow);
        Cv2.InRange(right, new Scalar(0, 254, 254), new Scalar(1, 255, 255), rightYellow);

        var leftYellowCount = Cv2.CountNonZero(leftYellow);
        var rightYellowCount = Cv2.CountNonZero(rightYellow);

        var lane = SelfdriveLane.Unknown;
        if (leftYellowCount > rightYellowCount * 1.5)
        {
            lane = SelfdriveLane.Right;
        }
        else if (rightYellowCount > leftYellowCount * 1.5)
        {
            lane = SelfdriveLane.Left;
        }

        return lane;
    }
}