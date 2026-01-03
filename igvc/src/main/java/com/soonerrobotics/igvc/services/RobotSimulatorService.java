package com.soonerrobotics.igvc.services;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.greenrobot.eventbus.EventBus;

import com.soonerrobotics.igvc.events.ImageEvent;
import com.soonerrobotics.sus.simulator.SimulatorLink;
import com.soonerrobotics.sus.simulator.packets.ImagePacket;
import com.soonerrobotics.sus.simulator.packets.Packet;

public class RobotSimulatorService extends SimulatorLink {
    private static final Logger logger = LogManager.getLogger(RobotSimulatorService.class);

    public RobotSimulatorService(String address, int port) {
        super(address, port);
    }

    @Override
    public void handlePacket(Packet<?> packet) {
        if (packet instanceof ImagePacket msg) {
            EventBus.getDefault().post(ImageEvent.fromPacket(msg));
            logger.info("Received image message with size: {} bytes", msg.getImageData().length);
        }
    }
}
