package com.soonerrobotics.igvc;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKey;
import static org.bytedeco.opencv.global.opencv_imgcodecs.imread;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;

public class OldMaintest {
    private static final Logger logger = LogManager.getLogger(OldMaintest.class);

    public void main(String[] args) throws Exception {
        Mat image = imread("input.jpg");
        if (image.empty()) {
            logger.error("Could not open or find the image!");
            return;
        }

        logger.info("Image loaded successfully with size: {}x{}", image.cols(), image.rows());

        // apply a blur for testing
        Mat blurredImage = new Mat();
        org.bytedeco.opencv.global.opencv_imgproc.GaussianBlur(image, blurredImage, new Size(15, 15), 0);

        // show it
        imshow("Blurred Image", blurredImage);
        waitKey(0);
    }
}