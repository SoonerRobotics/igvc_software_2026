using System.Runtime.InteropServices;
using igvc_csharp.MessageUtils;
using Messages;
using Microsoft.Extensions.Logging;
using OpenCvSharp;

namespace igvc_csharp.Utilities;

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
}