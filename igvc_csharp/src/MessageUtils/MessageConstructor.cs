using Google.FlatBuffers;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Utilities;
using Messages;
using Messages.Arc;
using OpenCvSharp;
using SocketCANSharp;

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
            TimeUtilities.Now(),
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

    public static ArcCommand CreateResponse(ArcCommand? command, byte[]? data = null)
    {
        if (command == null)
        {
            throw new ArgumentNullException(nameof(command), "Command cannot be null when creating a response.");
        }
        
        var builder = new FlatBufferBuilder(1024);
        var dataOffset = data != null ? ArcCommand.CreateDataVector(builder, data) : default(VectorOffset);
        var commandOffset = ArcCommand.CreateArcCommand(
            builder,
            TimeUtilities.Now(),
            command?.SequenceNumber ?? 0,
            ArcCommandPurpose.Response,
            command?.CommandId ?? 0,
            dataOffset
        );
        builder.Finish(commandOffset.Value);
        
        return ArcCommand.GetRootAsArcCommand(new ByteBuffer(builder.SizedByteArray()));
    }

    public static MessageWrapper CreateWrappedResponse(ArcCommand? command, byte[]? data = null)
    {
        var cmd = CreateResponse(command, data);
        return MessageWrapper.From(MessageType.CommandAck, cmd.ByteBuffer.ToSizedArray());
    }

    public static ArcCanFrame CreateArcCanFrame(CanFrame frame)
    {
        var builder = new FlatBufferBuilder(1024);
        var dataOffset = ArcCanFrame.CreateCanDataVector(builder, frame.Data);
        var arcCanFrameOffset = ArcCanFrame.CreateArcCanFrame(
            builder,
            TimeUtilities.Now(),
            0,
            frame.CanId,
            dataOffset
        );
        builder.Finish(arcCanFrameOffset.Value);
        
        return ArcCanFrame.GetRootAsArcCanFrame(new ByteBuffer(builder.SizedByteArray()));
    }

    public static ArcHistogram CreateHistogram(ColorUtilities.ColorRange range, int padding = 0)
    {
        var builder = new FlatBufferBuilder(1024);
        range = range.WithPadding(padding);
        var histogramOffset = ArcHistogram.CreateArcHistogram(
            builder,
            TimeUtilities.Now(),
            0,
            (float)range.MinHue,
            (float)range.MaxHue,
            (float)range.MinSaturation,
            (float)range.MaxSaturation,
            (float)range.MinValue,
            (float)range.MaxValue
        );
        builder.Finish(histogramOffset.Value);
        
        return ArcHistogram.GetRootAsArcHistogram(new ByteBuffer(builder.SizedByteArray()));
    }
}