package com.soonerrobotics.sus.simulator;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingDeque;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.soonerrobotics.sus.simulator.packets.Packet;

public abstract class SimulatorLink {
    private static final Logger logger = LogManager.getLogger(SimulatorLink.class);

    public static SimulatorLink INSTANCE;

    private static String m_Address;
    private static int m_Port;

    protected boolean m_Connected = false;

    // Socket and stream for communication
    private Socket m_Socket;
    private final BlockingQueue<Packet<?>> m_SendQueue = new LinkedBlockingDeque<>();
    private DataOutputStream m_OutputStream;
    private DataInputStream m_InputStream;

    // Threading
    private WriteThread m_Writer;
    private ReadThread m_Reader;
    private Thread m_ReadThread;
    private Thread m_WriteThread;
    private Thread m_ConnectThread;

    public SimulatorLink(String address, int port) {
        m_Address = address;
        m_Port = port;

        INSTANCE = this;
    }

    public void setConnected(boolean connected) {
        this.m_Connected = connected;
    }

    public void start() {
        m_ConnectThread = new Thread(() -> {
            while (!m_Connected) {
                try {
                    connect();
                    Thread.sleep(5000); // Wait before retrying
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    // logger.error("Connection thread interrupted", e);
                }
            }
        });
        m_ConnectThread.start();
    }

    public void connect() {
        try {
            m_Socket = new Socket(m_Address, m_Port);
            m_OutputStream = new DataOutputStream(m_Socket.getOutputStream());
            m_InputStream = new DataInputStream(m_Socket.getInputStream());

            m_Writer = new WriteThread(m_OutputStream, m_SendQueue);
            m_Reader = new ReadThread(m_InputStream);

            m_WriteThread = new Thread(m_Writer, "SimulatorWriteThread");
            m_ReadThread = new Thread(m_Reader, "SimulatorReadThread");

            m_WriteThread.start();
            m_ReadThread.start();

            logger.info("Connected to simulator at {}:{}", m_Address, m_Port);
        } catch (IOException e) {
            // logger.error("Failed to connect to simulator at {}:{}", m_Address, m_Port, e);
        }
    }

    public void disconnect() {
        try {
            if (m_Socket != null && !m_Socket.isClosed()) {
                m_Socket.close();
            }
            logger.info("Disconnected from simulator at {}:{}", m_Address, m_Port);
        } catch (IOException e) {
            logger.error("Error while disconnecting from simulator at {}:{}", m_Address, m_Port, e);
        }
    }

    public void sendPacket(Packet<?> packet) {
        try {
            m_SendQueue.put(packet);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            logger.error("Interrupted while sending packet to simulator", e);
        }
    }

    public abstract void handlePacket(Packet<?> packet);
}