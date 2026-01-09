using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision.Filters;

public sealed class BlurFilter : IFilter
{
    private readonly int _iterations;
    private readonly int _kernelSize;
    private readonly BlurMethod _method;

    public BlurFilter(
        int iterations,
        int kernelSize,
        BlurMethod method = BlurMethod.GaussianBlur)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(iterations);

        _iterations = iterations;
        _method = method;
        _kernelSize = ValidateKernel(kernelSize, method);
    }

    public Mat Apply(Mat frame)
    {
        for (var i = 0; i < _iterations; i++)
        {
            ApplyOnce(frame);
        }

        return frame;
    }

    private void ApplyOnce(Mat frame)
    {
        switch (_method)
        {
            case BlurMethod.GaussianBlur:
                Cv2.GaussianBlur(
                    frame,
                    frame,
                    new Size(_kernelSize, _kernelSize),
                    sigmaX: 0,
                    sigmaY: 0);
                break;

            case BlurMethod.MedianBlur:
                Cv2.MedianBlur(frame, frame, _kernelSize);
                break;

            case BlurMethod.BoxBlur:
                Cv2.Blur(
                    frame,
                    frame,
                    new Size(_kernelSize, _kernelSize));
                break;

            default:
                throw new ArgumentOutOfRangeException();
        }
    }

    private static int ValidateKernel(int k, BlurMethod method)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(k);

        var mustBeOdd = method is BlurMethod.GaussianBlur or BlurMethod.MedianBlur;
        if (mustBeOdd && k % 2 == 0)
        {
            throw new ArgumentException("Kernel size must be odd");
        }

        if (method == BlurMethod.MedianBlur && k < 3)
        {
            throw new ArgumentException("Median blur kernel must be >= 3");
        }

        return k;
    }

    public enum BlurMethod
    {
        BoxBlur,
        GaussianBlur,
        MedianBlur
    }
}
