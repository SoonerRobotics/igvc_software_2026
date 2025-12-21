package com.soonerrobotics;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.soonerrobotics.sus.simulator.SimulatorLink;
import com.soonerrobotics.sus.simulator.messages.ImageMessage;
import com.soonerrobotics.sus.simulator.messages.Message;

public class RobotSimulatorLink extends SimulatorLink {
    private static final Logger logger = LogManager.getLogger(RobotSimulatorLink.class);

    public RobotSimulatorLink(String address, int port) {
        super(address, port);
    }

    @Override
    public void handleMessage(Message<?> message) {
        if (message instanceof ImageMessage msg) {
            logger.info("Received image message with size: {} bytes", msg.getImageData().length);
        }
    }
}
