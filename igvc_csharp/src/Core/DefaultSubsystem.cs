using igvc_csharp.Core.Units;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Hardware.CanLayers;
using igvc_csharp.Utils;
using Messages;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Core;

[Subsystem("DefaultSubsystem")]
public class DefaultSubsystem : SubsystemBase
{
    private LatLng? mLastLatLng = null;

    public override Task Init(CancellationToken token)
    {
        Subscribe<CanFrameEvent>(OnCanReceived, token);
        SubscribeMessage<VectornavReport>(Utils.Messages.MessageType.VectorNav, OnVectornavReport, token);
        
        return base.Init(token);
    }

    private Task OnVectornavReport(VectornavReport report, CancellationToken token)
    {
        var ll = new LatLng(report.Latitude, report.Longitude);
        if (mLastLatLng == null)
        {
            mLastLatLng = ll;
            return Task.CompletedTask;
        }

        var distance = GeoUtils.LatLngDistance(mLastLatLng, ll);
        if (distance.To(DistanceUnit.Meters) < 0.1)
        {
            return Task.CompletedTask;
        }

        var heading = GeoUtils.EstimateHeading(mLastLatLng, ll);
        if (heading == null || !heading.HasValue)
        {
            return Task.CompletedTask;
        }
        
        Logger.LogDebug("Heading: {Heading}", heading.Value.To(AngleUnit.Degrees));
        var position = new RobotPosition(ll, heading.Value);
        BaseRobot.Instance?.SetRobotPosition(position);
        return Task.CompletedTask;
    }
    
    private Task OnCanReceived(CanFrameEvent cfe, CancellationToken token)
    {
        var frame = cfe.Frame;
        
        switch ((CanId)frame.CanId)
        {
            case CanId.MobilityStart:
                SetRobotMobility(true);
                break;
            
            case CanId.MobilityStop:
                SetRobotMobility(false);
                break;
            
            case CanId.EStop:
                IgvcRobot.Instance.SetEstopped(true);
                break;
        }
        
        return Task.CompletedTask;
    }
}