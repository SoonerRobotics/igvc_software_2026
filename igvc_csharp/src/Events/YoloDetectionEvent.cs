using igvc_csharp.Core;

namespace igvc_csharp.Events;

public sealed record YoloDetectionEvent(
    // generic fields
    string label,
    float confidence,
    
    // real world location relative to the robot camera
    float x,
    float y,
    float z
) : IRobotEvent;