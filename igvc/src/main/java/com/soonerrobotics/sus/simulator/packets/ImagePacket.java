package com.soonerrobotics.sus.simulator.packets;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

public class ImagePacket implements Packet<ImagePacket> {
    private static final PacketType PACKET_ID = PacketType.UNUSED;

    private final int mWidth;
    private final int mHeight;
    private final byte mPixelFormat;
    private final byte[] mImageData;
    private final int mImageIdentifier;

    public ImagePacket() {
        this(0, 0, (byte) 0, new byte[0], 0);
    }

    public ImagePacket(int width, int height, byte pixelFormat, byte[] imageData, int imageIdentifier) {
        this.mWidth = width;
        this.mHeight = height;
        this.mPixelFormat = pixelFormat;
        this.mImageData = imageData;
        this.mImageIdentifier = imageIdentifier;
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

    public int getImageIdentifier() {
        return mImageIdentifier;
    }

    @Override
    public void write(DataOutputStream output) throws IOException {
        output.writeInt(PACKET_ID.getValue());
        output.writeInt(mWidth);
        output.writeInt(mHeight);
        output.writeByte(mPixelFormat);
        output.writeInt(mImageData.length);
        output.write(mImageData);
        output.writeInt(mImageIdentifier);
    }

    @Override
    public ImagePacket read(DataInputStream input) throws IOException {
        int width = input.readInt();
        int height = input.readInt();
        byte pixelFormat = input.readByte();
        int dataLength = input.readInt();
        byte[] imageData = new byte[dataLength];
        input.readFully(imageData);
        int imageIdentifier = input.readInt();
        return new ImagePacket(width, height, pixelFormat, imageData, imageIdentifier);
    }
}
