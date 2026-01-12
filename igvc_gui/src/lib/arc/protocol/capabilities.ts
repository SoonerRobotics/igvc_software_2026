export enum Purpose
{
    Req = 0,
    Ack = 1
}

export enum VisionCapabilities
{
    // Raw Feeds
    FrontCamera = 1 << 0,
    
    // Altered Feeds
    HsvView = 1 << 1,
    YoloView = 1 << 2,
}

export enum Telemetry
{
    Gps = 1 << 0,
}

export enum Misc
{

}