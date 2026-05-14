using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware.CanLayers;

namespace igvc_csharp.Core;

[Subsystem("DefaultSubsystem")]
public class DefaultSubsystem : SubsystemBase
{
    public override Task Init(CancellationToken token)
    {
        Subscribe<CanFrameEvent>(OnCanReceived, token);
        
        return base.Init(token);
    }
    
    private Task OnCanReceived(CanFrameEvent cfe, CancellationToken token)
    {
        var frame = cfe.Frame;
        
        switch ((CanId)frame.CanId)
        {
            case CanId.MobilityStart:
                SetMobility(true);
                break;
            
            case CanId.MobilityStop:
                SetMobility(false);
                break;
            
            case CanId.EStop:
                IgvcRobot.Instance.SetEstopped(true);
                break;
        }
        
        return Task.CompletedTask;
    }
}