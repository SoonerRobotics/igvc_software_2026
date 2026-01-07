package com.soonerrobotics.igvc.services;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.greenrobot.eventbus.EventBus;

import com.soonerrobotics.igvc.events.ImageFrameEvent;
import com.soonerrobotics.sus.simulator.SimulatorLink;
import com.soonerrobotics.sus.utils.FlatBufferUtils.FlatBufferConverter;
import com.soonerrobotics.sus.utils.FlatBufferUtils.FlatBufferWrapper;

public class RobotSimulatorService extends SimulatorLink {
    private static final long SENDER_KEY = RobotSimulatorService.class.getCanonicalName().hashCode();
    private static final Logger LOGGER = LogManager.getLogger(RobotSimulatorService.class);

    public RobotSimulatorService(String address, int port) {
        super(address, port);
    }

    @Override
    public void handlePacket(FlatBufferWrapper wrapper) {
        LOGGER.debug("Received simulator packet of type {}", wrapper.messageType);

        switch (wrapper.messageType)
        {
            case IMAGE_FRAME -> EventBus.getDefault().post(new ImageFrameEvent(wrapper, SENDER_KEY));
            case ARC_LOG -> EventBus.getDefault().post(FlatBufferConverter.asArcLog(wrapper));
            default -> LOGGER.warn("Received unknown simulator packet of type {}", wrapper.messageType);
        }
    }
}
