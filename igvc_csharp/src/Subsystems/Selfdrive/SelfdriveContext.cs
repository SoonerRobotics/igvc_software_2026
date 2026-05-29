
using igvc_csharp.Events;
using igvc_csharp.src.subsystems.selfdrive;
using igvc_csharp.Subsystems.Hardware;

namespace igvc_csharp.src.Subsystems.selfdrive;

public class SelfdriveContext
{
    public CanbusSubsystem canbus;

    public SelfdriveLane CurrentLane;

    public Dictionary<string, YoloDetectionEvent> YoloDetections;

    // left, right, center camera
    // curent lane
    // canbus
}