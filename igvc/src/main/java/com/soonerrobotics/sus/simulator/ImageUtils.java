package com.soonerrobotics.sus.simulator;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import com.soonerrobotics.sus.simulator.packets.ImagePacket;

public class ImageUtils {
    public static Mat convertToMat(ImagePacket packet) {
        Mat mat;

        switch (packet.getPixelFormat()) {
            case PixelFormat.RGB8 -> {
                Mat rgb = new Mat(packet.getHeight(), packet.getWidth(), CvType.CV_8UC3);
                rgb.put(0, 0, packet.getImageData());

                mat = new Mat();
                Imgproc.cvtColor(rgb, mat, Imgproc.COLOR_RGB2BGR);
                return mat;
            }

            case PixelFormat.GRAY8 -> {
                mat = new Mat(packet.getHeight(), packet.getWidth(), CvType.CV_8UC1);
                mat.put(0, 0, packet.getImageData());
                return mat;
            }

            default -> throw new IllegalArgumentException("Unsupported pixel format: " + packet.getPixelFormat());
        }
    }
}
