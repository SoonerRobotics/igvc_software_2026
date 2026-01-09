namespace igvc_csharp.Subsystems.Arc;

public static class Capabilities
{
    public enum Purpose : uint
    {
        Req = 0,
        Ack = 1
    }
    
    [Flags]
    public enum Vision : uint
    {
        // Raw Feeds
        FrontCamera = 1 << 0,
        
        // Altered Feeds
        HsvView = 1 << 1,
        YoloView = 1 << 2,
    }

    [Flags]
    public enum Telemetry : uint
    {
        Gps = 1 << 0
    }
    
    [Flags]
    public enum Misc : uint
    {
        
    }
}