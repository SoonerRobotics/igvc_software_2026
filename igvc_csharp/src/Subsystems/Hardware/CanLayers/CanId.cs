namespace igvc_csharp.Subsystems.Hardware.CanLayers;

public enum CanId : uint
{
    EStop = 0x0,
    MobilityStop = 0x1,
    MobilityStart = 0x9,
    
    MotorCommand = 0xA,
    MotorOdometry = 0xE,
    SafetyLights = 0x14,
    HubTelemetry = 0x15,
    
    // Current Sensor
    SSD_COMMAND = 0x3FA,
    SSD_CURRENT = 0x3F1,
    SSD_VOLTAGE  = 0x3F3,
}