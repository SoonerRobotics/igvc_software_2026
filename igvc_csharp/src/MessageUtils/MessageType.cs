namespace igvc_csharp.MessageUtils;

public enum MessageType : ushort
{
    // 2026
    ImageFrame = 0x01,
    ArcLog = 0x02,
    Gps = 0x03,
    Metric = 0x04,
    MetricHistory = 0x05,
    
    // Reserved Stuff
    CapabilityReq = 60_000,
    CapabilityAck = 60_001,
    CommandReq = 60_002,
    CommandAck = 60_003,
}