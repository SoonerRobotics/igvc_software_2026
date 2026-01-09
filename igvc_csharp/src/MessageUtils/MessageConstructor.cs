using Google.FlatBuffers;
using igvc_csharp.Subsystems.Arc;
using Messages;
using Messages.Arc;

namespace igvc_csharp.MessageUtils;

public static class MessageConstructor
{
    // Image Frame
    
    public static ImageFrame CreateImageFrame(uint width, uint height, string identifier, byte[] data)
    {
        var builder = new FlatBufferBuilder(1024);
        var encodingOffset = builder.CreateString("jpeg");
        var identifierOffset = builder.CreateString(identifier);
        var jpegOffset = ImageFrame.CreateImageDataVector(builder, data);
        var imageOffset = ImageFrame.CreateImageFrame(
            builder,
            (ulong)System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            0,
            width,
            height,
            encodingOffset,
            identifierOffset,
            jpegOffset
        );
        builder.Finish(imageOffset.Value);

        return ImageFrame.GetRootAsImageFrame(new ByteBuffer(builder.SizedByteArray()));
    }

    public static ImageFrame ModifyImageFrame(ImageFrame frame, byte[] data, string identifier = "")
    {
        return CreateImageFrame(frame.Width, frame.Height, identifier == "" ? frame.Identifier : identifier, data);
    }
    
    // ArcCapability

    public static ArcCapability CreateArcCapabilityAck(ArcCapability capability)
    {
        var builder = new FlatBufferBuilder(1024);
        var capabilityOffset = ArcCapability.CreateArcCapability(
            builder,
            (ulong)System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            0,
            (uint)Capabilities.Purpose.Ack,
            capability.VisionCapabilities,
            capability.TelemetryCapabilities,
            capability.MiscCapabilities
        );
        builder.Finish(capabilityOffset.Value);
        
        return ArcCapability.GetRootAsArcCapability(new ByteBuffer(builder.SizedByteArray()));
    }
}