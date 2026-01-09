using OpenCvSharp;

namespace igvc_csharp.Subsystems.Vision;

public interface IFilter
{
    public Mat Apply(Mat frame);
}