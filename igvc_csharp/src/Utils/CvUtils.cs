using System.IO.Compression;
using Google.FlatBuffers;
using igvc_csharp.Utils.Messages;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Utils;

public class CvUtils
{
    private static ILogger Logger = Logging.From<CvUtils>();

    public static Mat AsMat(ImageFrame frame)
    {
        var byts = frame.GetImageDataArray();
        return Cv2.ImDecode(byts, ImreadModes.Color);
    }

    public static Mat AsDepthMat(ZedFrame frame)
    {
        return ProcessDepthMessage(frame);
    }

    public static Mat ProcessDepthMessage(ZedFrame frame)
    {
        int w          = frame.Width;
        int h          = frame.Height;
        byte[] compressed = frame.GetDataArray(); // now bytes not floats

        // Decompress
        byte[] floatBytes;
        using (var ms  = new MemoryStream(compressed))
        using (var gz  = new DeflateStream(ms, CompressionMode.Decompress))
        using (var p = new MemoryStream())
        {
            gz.CopyTo(p);
            floatBytes = p.ToArray();
        }

        // Reinterpret as float[]
        float[] data = new float[floatBytes.Length / sizeof(float)];
        Buffer.BlockCopy(floatBytes, 0, data, 0, floatBytes.Length);

        var mat = new Mat(h, w, MatType.CV_32FC1);
        unsafe
        {
            fixed (float* src = data)
                Buffer.MemoryCopy(
                    src,
                    mat.DataPointer,
                    (long)w * h * sizeof(float),
                    (long)w * h * sizeof(float)
                );
        }

        return mat;
    }
    
    public static float SampleDepth(Mat depthMat, Rect bounding)
    {
        // Sample a small region around the box center rather than a single pixel
        // to avoid hitting invalid (0) pixels
        int cx = bounding.X + bounding.Width  / 2;
        int cy = bounding.Y + bounding.Height / 2;

        int sampleRadius = 5;
        int x0 = Math.Max(0, cx - sampleRadius);
        int y0 = Math.Max(0, cy - sampleRadius);
        int x1 = Math.Min(depthMat.Width  - 1, cx + sampleRadius);
        int y1 = Math.Min(depthMat.Height - 1, cy + sampleRadius);

        float sum   = 0f;
        int   count = 0;

        for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
        {
            float d = depthMat.At<float>(y, x);
            if (d >= 0.2f && d <= 20f) { sum += d; count++; }
        }

        return count > 0 ? sum / count : float.NaN;
    }

    public static Mat AsColorizedMat(ZedFrame frame, float minMeters = 0.2f, float maxMeters = 20f)
    {
        var mat = ProcessDepthMessage(frame);
        
        var normalized = new Mat();
        mat.ConvertTo(normalized, MatType.CV_8UC1, 255.0 / (maxMeters - minMeters), -minMeters * 255.0 / (maxMeters - minMeters));

        // Apply colormap
        var colorized = new Mat();
        Cv2.ApplyColorMap(normalized, colorized, ColormapTypes.Jet);

        // Mask out invalid pixels (where depth was 0)
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

    public static Mat CloneMat(Mat mat)
    {
        return mat.Clone();
    }

    public static MessageWrapper BuildWrapper(uint width, uint height, string id, byte[] frame)
    {
        var newFrame = MessageConstructor.CreateImageFrame(
            width,
            height,
            id,
            frame
        );

        var wrappedFrame = MessageWrapper.From(
            MessageType.ImageFrame,
            newFrame.ByteBuffer.ToFullArray()
        );
        return wrappedFrame;
    }

    // Histogram Stuff

    /// <summary>
    /// Extracts the HSV color range from a mat
    /// </summary>
    /// <param name="mat"></param>
    /// <param name="rect"></param>
    /// <returns></returns>
    public static ColorUtils.ColorRange ExtractHsvRange(Mat mat, Rect rect)
    {
        using var roi = new Mat(mat, rect);

        // Convert RGB → HSV
        using var hsv = new Mat();
        Cv2.CvtColor(roi, hsv, ColorConversionCodes.RGB2HSV);

        // Split channels
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
            {
                c.Dispose();
            }
        }
    }
}
