package com.soonerrobotics.igvc.events;

import java.nio.ByteBuffer;

import com.soonerrobotics.flatbuffers.ImageFrame;
import com.soonerrobotics.sus.utils.FlatBufferUtils;

public final class ImageFrameEvent {
    private final byte[] data;
    private long senderKey;

    public ImageFrameEvent(byte[] data, long senderKey) {
        this.data = data;
        this.senderKey = senderKey;
    }

    public ImageFrameEvent(FlatBufferUtils.FlatBufferWrapper wrapper, long senderKey) {
        this(wrapper.getPayload(), senderKey);
    }

    public boolean isSender(long key) {
        return this.senderKey == key;
    }

    public byte[] data() {
        return data;
    }

    public ImageFrame frame() {
        return ImageFrame.getRootAsImageFrame(ByteBuffer.wrap(data));
    }
}