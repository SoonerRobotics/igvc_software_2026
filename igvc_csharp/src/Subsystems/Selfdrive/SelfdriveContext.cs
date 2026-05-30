
using igvc_csharp.Events;
using igvc_csharp.src.subsystems.selfdrive;
using igvc_csharp.Subsystems.Hardware;
using OpenCvSharp;

namespace igvc_csharp.src.Subsystems.selfdrive;

public class SelfdriveContext
{
    public CanbusSubsystem canbus;

    public SelfdriveLane CurrentLane;

    public Dictionary<string, YoloDetectionEvent> YoloDetections;

    public double whiteLaneDistance;
    public Mat? lastZedFrame;
    public Mat? lastCenterFrame;

    // left, right, center camera
    // curent lane
    // canbus
}