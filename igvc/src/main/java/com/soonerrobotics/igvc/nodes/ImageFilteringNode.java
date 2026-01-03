package com.soonerrobotics.igvc.nodes;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.greenrobot.eventbus.Subscribe;

import com.soonerrobotics.igvc.events.ImageEvent;

public class ImageFilteringNode extends Node {
    private static final Logger logger = LogManager.getLogger(ImageFilteringNode.class);

    @Subscribe
    public void onImageReceived(ImageEvent event)
    {
        logger.info("Image received with dimensions: ({} x {}) with identifier: {}", event.getWidth(), event.getHeight(), event.getImageIdentifier());
    }
}