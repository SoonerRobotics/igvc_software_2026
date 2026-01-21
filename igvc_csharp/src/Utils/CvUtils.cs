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