package com.soonerrobotics.sus.simulator;

import java.io.DataInputStream;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.soonerrobotics.sus.simulator.messages.MessageType;

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
                MessageType messageType = MessageType.fromInt(m_InputStream.readInt());

                switch (messageType) {
                    case CONNECT -> {
                        SimulatorLink.INSTANCE.setConnected(m_Running);
                    }
                    default -> {
                        logger.warn("Received unknown or unhandled message type: " + messageType);
                    }
                }
            } catch (Exception e) {
                logger.error("Error while reading message from simulator", e);
            }
        }
    }

    public void stop() {
        m_Running = false;
    }
}
