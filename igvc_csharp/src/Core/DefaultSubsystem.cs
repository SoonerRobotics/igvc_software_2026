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
    private RobotPosition? mLastPosition = null;

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
            Logger.LogWarning("Received first VN report");
            mLastLatLng = ll;
            return Task.CompletedTask;
        }

        var distance = GeoUtils.LatLngDistance(mLastLatLng, ll);
        // Logger.LogDebug("Moved: {meters}", distance.To(DistanceUnit.Meters));
        if (distance.To(DistanceUnit.Feet) < 1.0)
        {
            // Still update the position, but don't update the heading since we can't get a good estimate of it
            var h = mLastPosition?.Heading;
            if (h != null)
            {
                var p = new RobotPosition(ll, h.Value);
                BaseRobot.Instance?.SetRobotPosition(p);
                mLastPosition = p;
            }

            return Task.CompletedTask;
        }

        var heading = GeoUtils.EstimateHeading(mLastLatLng, ll);
        if (heading == null || !heading.HasValue)
        {
            return Task.CompletedTask;
        }
        
        // Logger.LogDebug("Robot Moved: {hd}", heading);
        var position = new RobotPosition(ll, heading.Value);
        BaseRobot.Instance?.SetRobotPosition(position);
        mLastLatLng = ll;
        mLastPosition = position;
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