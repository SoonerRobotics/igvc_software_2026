namespace igvc_csharp.CanSpec;

public enum CanId : uint
{
    EStop = 0x0,
    MobilityStop = 0x1,
    MobilityStart = 0x9,
    
    MotorCommand = 0xA,
    MotorOdometry = 0xE,
    SafetyLights = 0x14,
    HubTelemetry = 0x15,
}