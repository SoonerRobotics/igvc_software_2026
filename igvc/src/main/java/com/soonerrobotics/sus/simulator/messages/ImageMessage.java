package com.soonerrobotics.sus.simulator.messages;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

// This should not be used, instead it should be used as a template for other messages (e.g. CameraImageMessage)
public class ImageMessage implements Message<ImageMessage> {
    private static final MessageType MESSAGE_ID = MessageType.UNUSED;

    private final int mWidth;
    private final int mHeight;
    private final byte mPixelFormat;
    private final byte[] mImageData;

    public ImageMessage(int width, int height, byte pixelFormat, byte[] imageData) {
        this.mWidth = width;
        this.mHeight = height;
        this.mPixelFormat = pixelFormat;
        this.mImageData = imageData;
    }

    public int getWidth() {
        return mWidth;
    }

    public int getHeight() {
        return mHeight;
    }

    public byte getPixelFormat() {
        return mPixelFormat;
    }

    public byte[] getImageData() {
        return mImageData;
    }

    @Override
    public void write(DataOutputStream output) throws IOException {
        output.writeInt(MESSAGE_ID.getValue());
        output.writeInt(mWidth);
        output.writeInt(mHeight);
        output.writeByte(mPixelFormat);
        output.writeInt(mImageData.length);
        output.write(mImageData);
    }

    @Override
    public ImageMessage read(DataInputStream input) throws IOException {
        int width = input.readInt();
        int height = input.readInt();
        byte pixelFormat = input.readByte();
        int dataLength = input.readInt();
        byte[] imageData = new byte[dataLength];
        input.readFully(imageData);
        return new ImageMessage(width, height, pixelFormat, imageData);
    }
}
