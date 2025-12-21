package com.soonerrobotics.sus.simulator;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import com.soonerrobotics.sus.simulator.messages.ImageMessage;

public class ImageUtils {
    public static Mat convertToMat(ImageMessage message) {
        Mat mat;

        switch (message.getPixelFormat()) {
            case PixelFormat.RGB8 -> {
                Mat rgb = new Mat(message.getHeight(), message.getWidth(), CvType.CV_8UC3);
                rgb.put(0, 0, message.getImageData());

                mat = new Mat();
                Imgproc.cvtColor(rgb, mat, Imgproc.COLOR_RGB2BGR);
                return mat;
            }

            case PixelFormat.GRAY8 -> {
                mat = new Mat(message.getHeight(), message.getWidth(), CvType.CV_8UC1);
                mat.put(0, 0, message.getImageData());
                return mat;
            }

            default -> throw new IllegalArgumentException("Unsupported pixel format: " + message.getPixelFormat());
        }
    }
}
