package com.soonerrobotics.sus.simulator;

import java.io.DataInputStream;
import java.io.IOException;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.soonerrobotics.sus.simulator.packets.ImagePacket;
import com.soonerrobotics.sus.simulator.packets.MotorFeedbackPacket;
import com.soonerrobotics.sus.simulator.packets.PacketType;

public class ReadThread implements Runnable {
    private static final Logger logger = LogManager.getLogger(ReadThread.class);

    private final DataInputStream m_InputStream;
    private volatile boolean m_Running = true;

    public ReadThread(DataInputStream inputStream) {
        this.m_InputStream = inputStream;
    }

    @Override
    public void run() {
        while (m_Running) {
            try {
                PacketType packetType = PacketType.fromInt(m_InputStream.readInt());

                switch (packetType) {
                    case R_MOTORFEEDBACK_2026 -> {
                        SimulatorLink.INSTANCE.handlePacket(new MotorFeedbackPacket().read(m_InputStream));
                    }
                    case R_CAMERAFRAME_2026 -> {
                        SimulatorLink.INSTANCE.handlePacket(new ImagePacket().read(m_InputStream));
                    }
                    default -> {
                        logger.warn("Received unknown or unhandled packet type: " + packetType);
                    }
                }
            } catch (IOException e) {
                logger.error("Error while reading packet from simulator", e);
                SimulatorLink.INSTANCE.setConnected(false);
                m_Running = false;
            }
        }
    }

    public void stop() {
        m_Running = false;
    }
}
