package com.soonerrobotics.arc;

import java.nio.ByteBuffer;

import com.soonerrobotics.arc.messages.MessageType;
import com.soonerrobotics.igvc.events.ImageEvent;

public class ArcTransformers {
    public static ByteBuffer transformImageEvent(ImageEvent event) {
        // MessageType (1 byte) + Image Length (4 bytes) + Image Identifier (1 byte) + Image Data (variable length)
        ByteBuffer buffer = ByteBuffer.allocate(1 + 4 + 1 + event.getImageData().length);

        buffer.put((byte) MessageType.IMAGE_MESSAGE);
        buffer.putInt(event.getImageData().length);
        buffer.put((byte) event.getImageIdentifier());
        buffer.put(event.getImageData());
        buffer.flip();

        return buffer;
    }
}
