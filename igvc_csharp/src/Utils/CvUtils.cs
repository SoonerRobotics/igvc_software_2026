using System.Buffers;
using System.Runtime.InteropServices;
using Google.FlatBuffers;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;
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

    public static float SampleDepth(Mat depthMat, Rect bounding)
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
    /// </summary>
    public static ColorUtils.ColorRange ExtractHsvRange(Mat mat, Rect rect)
    {
        using var roi = new Mat(mat, rect);
        using var hsv = new Mat();
        Cv2.CvtColor(roi, hsv, ColorConversionCodes.RGB2HSV);
        Cv2.Split(hsv, out var channels);

        try
        {
            Cv2.MinMaxLoc(channels[0], out var minH, out double maxH);
            Cv2.MinMaxLoc(channels[1], out var minS, out double maxS);
            Cv2.MinMaxLoc(channels[2], out var minV, out double maxV);

            return ColorUtils.ColorRange.From(
                ColorUtils.Color.FromHsv((int)minH, (int)minS, (int)minV),
                ColorUtils.Color.FromHsv((int)maxH, (int)maxS, (int)maxV)
            );
        }
        finally
        {
            foreach (var c in channels)
                c.Dispose();
        }
    }
}