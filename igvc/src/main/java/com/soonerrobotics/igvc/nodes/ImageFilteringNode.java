package com.soonerrobotics.igvc.nodes;

import java.time.Instant;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import static org.bytedeco.opencv.global.opencv_imgproc.cvtColor;
import org.bytedeco.opencv.opencv_core.Mat;
import org.greenrobot.eventbus.EventBus;
import org.greenrobot.eventbus.Subscribe;
import org.greenrobot.eventbus.ThreadMode;

import com.google.flatbuffers.FlatBufferBuilder;
import com.soonerrobotics.flatbuffers.ImageFrame;
import com.soonerrobotics.igvc.events.ImageFrameEvent;
import com.soonerrobotics.sus.utils.FlatBufferUtils;
import com.soonerrobotics.sus.utils.ImageUtilities;

public class ImageFilteringNode extends Node {
    private static final long SENDER_KEY = ImageFilteringNode.class.getCanonicalName().hashCode();
    private static final Logger LOGGER = LogManager.getLogger(ImageFilteringNode.class);
    private static final ExecutorService IMAGE_EXECUTOR = Executors.newSingleThreadExecutor();

    @Subscribe(threadMode = ThreadMode.POSTING)
    public void onImageReceived(ImageFrameEvent event) {
        // Prevent processing our own sent events
        if (event.isSender(SENDER_KEY)) {
            return;
        }

        IMAGE_EXECUTOR.submit(() -> process(event));
    }

    private void process(ImageFrameEvent event)
    {
        final ImageFrame frame = event.frame();
        Mat mat = ImageUtilities.convertToMat(frame);

        try {
            // // Apply grayscale to confirm it all works
            cvtColor(mat, mat, org.bytedeco.opencv.global.opencv_imgproc.COLOR_BGR2GRAY);

            byte[] processedImageData = ImageUtilities.convertToBytes(mat);

            FlatBufferBuilder builder = new FlatBufferBuilder(1024);

            var encodingOffset = builder.createString(frame.encoding());
            var identifierOffset = builder.createString("top_camera");
            var jpegOffset = ImageFrame.createImageDataVector(builder, processedImageData);

            var frameIdentifier = ImageFrame.createImageFrame(
                    builder,
                    Instant.now().toEpochMilli(),
                    0,
                    frame.width(),
                    frame.height(),
                    encodingOffset,
                    identifierOffset,
                    jpegOffset);
            builder.finish(frameIdentifier);

            var wrappedFrame = FlatBufferUtils.FlatBufferWrapper.create(
                    FlatBufferUtils.FlatBufferType.IMAGE_FRAME,
                    builder.sizedByteArray());
            if (wrappedFrame.isError()) {
                LOGGER.error("Failed to wrap processed ImageFrame: " + wrappedFrame.getError());
                return;
            }

            EventBus.getDefault().post(new ImageFrameEvent(wrappedFrame.get(), SENDER_KEY));
        } finally {
            mat.release();
        }
    }
}