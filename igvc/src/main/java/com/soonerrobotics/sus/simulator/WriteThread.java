package com.soonerrobotics.sus.simulator;

import java.io.DataOutputStream;
import java.io.IOException;
import java.util.concurrent.BlockingQueue;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.soonerrobotics.sus.simulator.packets.Packet;

public class WriteThread implements Runnable {
    private static final Logger logger = LogManager.getLogger(WriteThread.class);

    private final DataOutputStream m_OutputStream;
    private final BlockingQueue<Packet<?>> m_MessageQueue;
    private volatile boolean m_Running = true;

    public WriteThread(DataOutputStream outputStream, BlockingQueue<Packet<?>> messageQueue) {
        this.m_OutputStream = outputStream;
        this.m_MessageQueue = messageQueue;
    }

    @Override
    public void run() {
        while (m_Running) {
            try {
                Packet<?> message = m_MessageQueue.take();
                message.write(m_OutputStream);
                m_OutputStream.flush();

                logger.debug("Sent message: " + message.getClass().getSimpleName());
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } catch (IOException e) {
                logger.error("Error while writing message to simulator", e);
                m_Running = false;
                SimulatorLink.INSTANCE.setConnected(false);
            }
        }
    }

    public void stop() {
        m_Running = false;
    }
}
