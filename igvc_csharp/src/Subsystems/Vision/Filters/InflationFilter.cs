using OpenCvSharp;
using System;
using System.Collections.Generic;

namespace igvc_csharp.Subsystems.Vision.Filters;

/// </summary>
public class InflationFilter(float maxRadius = 0.54f, float noGoPercent = 0.63f) : IFilter
{
    private readonly List<(int dx, int dy, double radius)> _ring = BuildRing(maxRadius, noGoPercent);

    private static List<(int dx, int dy, double radius)> BuildRing(float maxRadius, float noGoPercent)
    {
        const float horizontalFov = 3f;
        const float mapResF = 80f;
        float scale = horizontalFov / mapResF;

        int maxPx = (int)(maxRadius / scale);
        int noGoPx = (int)(maxRadius * noGoPercent / scale);

        var ring = new List<(int, int, double)>
        {
            (0, 0, 0)
        };
        for (int dx = -maxPx; dx <= maxPx; dx++)
        {
            for (int dy = -maxPx; dy <= maxPx; dy++)
            {
                double r = Math.Sqrt(dx * dx + dy * dy);
                if (r >= noGoPx && r < maxPx)
                    ring.Add((dx, dy, r));
            }
        }

        return ring;
    }

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

        int w = gray.Cols;
        int h = gray.Rows;

        double noGoPx = 0, maxPx = 0;
        foreach (var (_, _, r) in _ring)
        {
            if (r > 0 && noGoPx == 0) noGoPx = r;
            if (r > maxPx) maxPx = r;
        }

        var output = new Mat(h, w, MatType.CV_8UC1);
        gray.CopyTo(output);

        var srcIndexer = gray.GetGenericIndexer<byte>();
        var dstIndexer = output.GetGenericIndexer<byte>();
        for (int x = 0; x < w; x++)
        {
            for (int y = 0; y < h; y++)
            {
                if (srcIndexer[y, x] == 0) continue;

                foreach (var (dx, dy, radius) in _ring)
                {
                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;

                    byte newVal;
                    if (radius <= noGoPx)
                    {
                        newVal = 255;
                    }
                    else
                    {
                        double t = (radius - noGoPx) / (maxPx - noGoPx);
                        newVal = (byte)(255 * (1.0 - t));
                    }

                    if (dstIndexer[ny, nx] < newVal)
                    {
                        dstIndexer[ny, nx] = newVal;
                    }
                }
            }
        }

        gray.Dispose();

        if (!wasColor) return output;

        var result = new Mat();
        Cv2.CvtColor(output, result, ColorConversionCodes.GRAY2BGR);
        output.Dispose();
        return result;
    }
}