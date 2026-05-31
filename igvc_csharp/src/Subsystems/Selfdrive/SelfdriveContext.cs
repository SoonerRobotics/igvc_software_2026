using igvc_csharp.Events;
using igvc_csharp.src.subsystems.selfdrive;
using igvc_csharp.Subsystems.Hardware;
using OpenCvSharp;

namespace igvc_csharp.src.Subsystems.selfdrive;

public class SelfdriveContext
{
    public required CanbusSubsystem Canbus { get; init; }
    public SelfdriveLane CurrentLane { get; set; }

    // Field (not property) so SwapFrame can pass it as ref.
    public Mat? LastZedFrame;
    public Mat? LastCenterFrame;
    public Mat? LastFilteredFrame;

    // Private setter so the required-init pattern works.
    public Dictionary<string, YoloDetectionEvent> YoloDetections { get; set; } = new();
    public YoloDetectionTracker DetectionTracker { get; } = new(framesToConfirm: 3, framesToLose: 5);
    public double WhiteLaneDistance { get; set; }
}