export enum MessageType {
    // 2026
    ImageFrame = 0x01,
    DepthFrame = 0x06,
    CanFrame = 0x07,
    VectorNav = 0x08,
    ZedFrame = 0x09,
    Waypoint = 0x10,
    AudibleFeedback = 0x11,

    // Reserved
    CapabilityReq = 60_000,
    CapabilityAck = 60_001,
    CommandReq = 60_002,
    CommandAck = 60_003,
    ArcData = 60_004,
}