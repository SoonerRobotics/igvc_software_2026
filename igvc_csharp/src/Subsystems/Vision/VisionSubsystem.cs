using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.MessageUtils;
using igvc_csharp.Subsystems.Vision;
using igvc_csharp.Subsystems.Vision.Filters;
using igvc_csharp.Utilities;
using Messages;
using OpenCvSharp;

namespace igvc_csharp.Subsystems;

[Subsystem("VisionSubsystem")]
public class VisionSubsystem : SubsystemBase
{
    private List<IFilter> _filters = new();
    
    public override Task Init(CancellationToken token)
    {
        AddFilters();
        
        SubscribeImage(
            "front_view",
            OnImageReceived,
            token
        );

        return Task.CompletedTask;
    }

    private void AddFilters()
    {
        // Get rid of super fine grained details to assist our hsv filter
        _filters.Add(new BlurFilter(5, 3, BlurFilter.BlurMethod.BoxBlur));
        
        // Ground hsv filter
        _filters.Add(new HsvFilter(Constants.VisionSubsystem.GroundThreshold));
    }

    private Task OnImageReceived(ImageFrame frame, CancellationToken token)
    {
        var mat = CvUtils.AsMat(frame);
        
        foreach (var f in _filters)
        {
            f.Apply(mat);
        }
        
        var frameBytes = CvUtils.FromMat(mat);
        var newFrame = MessageConstructor.CreateImageFrame(640, 480, "yolo_view", frameBytes);
        var wrappedFrame = MessageWrapper.From(MessageType.ImageFrame, newFrame.ByteBuffer.ToFullArray());
        EventBus.Write(new MessageWrapperEvent(wrappedFrame));
        return Task.CompletedTask;
    }
}