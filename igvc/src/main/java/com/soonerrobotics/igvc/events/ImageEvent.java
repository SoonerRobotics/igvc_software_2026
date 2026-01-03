package com.soonerrobotics.igvc.events;

import com.soonerrobotics.sus.simulator.packets.ImagePacket;

public class ImageEvent {
    private final int mWidth;
    private final int mHeight;
    private final byte mPixelFormat;
    private final byte[] mImageData;
    private final int mImageIdentifier;

    public ImageEvent(int width, int height, byte pixelFormat, byte[] imageData, int imageIdentifier) {
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

    public static ImageEvent fromPacket(ImagePacket packet)
    {
        return new ImageEvent(
            packet.getWidth(),
            packet.getHeight(),
            packet.getPixelFormat(),
            packet.getImageData(),
            packet.getImageIdentifier()
        );
    }
}