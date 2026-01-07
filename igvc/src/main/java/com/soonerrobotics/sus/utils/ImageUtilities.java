package com.soonerrobotics.sus.utils;

import java.nio.ByteBuffer;

import org.bytedeco.javacpp.BytePointer;
import static org.bytedeco.opencv.global.opencv_imgcodecs.IMREAD_COLOR;
import static org.bytedeco.opencv.global.opencv_imgcodecs.imdecode;
import static org.bytedeco.opencv.global.opencv_imgcodecs.imencode;
import org.bytedeco.opencv.opencv_core.Mat;

import com.soonerrobotics.flatbuffers.ImageFrame;

public class ImageUtilities {
    public static Mat convertToMat(ImageFrame frame) {
        int len = frame.imageDataLength();

        if (len == 0) {
            throw new IllegalArgumentException("ImageFrame contains no image data");
        }

        byte[] imageData = new byte[len];
        ByteBuffer imageDataBuffer = frame.imageDataAsByteBuffer();
        imageDataBuffer.get(imageData, 0, len);

        Mat encoded = new Mat(1, len, org.bytedeco.opencv.global.opencv_core.CV_8UC1);
        encoded.data().put(imageData);

        Mat decoded = imdecode(encoded, IMREAD_COLOR);

        encoded.release();

        if (decoded == null || decoded.empty()) {
            throw new RuntimeException("imdecode failed: invalid image bytes");
        }

        return decoded;
    }

    public static byte[] convertToBytes(Mat mat) {
        BytePointer out = new BytePointer();

        imencode(".jpg", mat, out);

        byte[] bytes = new byte[(int) out.limit()];
        out.get(bytes);

        out.deallocate();

        return bytes;
    }
}
