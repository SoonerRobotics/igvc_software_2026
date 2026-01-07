package com.soonerrobotics.sus.simulator;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.Closeable;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.soonerrobotics.igvc.Robot;
import com.soonerrobotics.sus.utils.FlatBufferUtils;

public abstract class SimulatorLink {

    private static final Logger logger = LogManager.getLogger(SimulatorLink.class);

    private final String address;
    private final int port;

    private final BlockingQueue<byte[]> sendQueue = new LinkedBlockingQueue<>();
    private final AtomicBoolean running = new AtomicBoolean(false);
    private final AtomicBoolean connected = new AtomicBoolean(false);

    private Socket socket;
    private DataInputStream in;
    private DataOutputStream out;

    private Thread readerThread;
    private Thread writerThread;
    private Thread connectThread;

    protected SimulatorLink(String address, int port) {
        this.address = address;
        this.port = port;
    }

    public boolean isConnected() {
        return connected.get();
    }

    public void start() {
        running.set(true);

        connectThread = new Thread(this::connectionLoop, "SimulatorConnectThread");
        connectThread.start();
    }

    private void connectionLoop() {
        while (running.get() && !Robot.isShuttingDown.get()) {
            if (!connected.get()) {
                try {
                    connect();
                } catch (IOException e) {
                    logger.warn("Failed to connect, retrying...", e);
                    sleep(1000);
                }
            }

            sleep(500);
        }
    }

    private synchronized void connect() throws IOException {
        socket = new Socket(address, port);
        socket.setTcpNoDelay(true);

        in = new DataInputStream(new BufferedInputStream(socket.getInputStream()));
        out = new DataOutputStream(new BufferedOutputStream(socket.getOutputStream()));

        connected.set(true);

        readerThread = new Thread(new ReadWorker(), "SimulatorReadThread");
        writerThread = new Thread(new WriteWorker(), "SimulatorWriteThread");

        readerThread.start();
        writerThread.start();

        logger.info("Connected to simulator at {}:{}", address, port);
    }

    public synchronized void disconnect() {
        if (!connected.getAndSet(false)) {
            return;
        }

        logger.info("Disconnecting from simulator");

        closeQuietly(socket);
        closeQuietly(in);
        closeQuietly(out);
    }

    public void shutdown() {
        running.set(false);
        disconnect();
    }

    public void sendPacket(FlatBufferUtils.FlatBufferWrapper packet) {
        if (!connected.get()) {
            return;
        }

        sendQueue.offer(packet.toByteArray());
    }

    protected abstract void handlePacket(FlatBufferUtils.FlatBufferWrapper packet);

    private static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ignored) {
        }
    }

    private static void closeQuietly(Closeable c) {
        try {
            if (c != null)
                c.close();
        } catch (IOException ignored) {
        }
    }

    private class WriteWorker implements Runnable {
        @Override
        public void run() {
            try {
                while (connected.get()) {
                    byte[] msg = sendQueue.take();
                    out.write(msg);
                    out.flush();
                }
            } catch (InterruptedException ignored) {
            } catch (IOException e) {
                logger.error("Writer error", e);
            } finally {
                disconnect();
            }
        }
    }

    private class ReadWorker implements Runnable {
        @Override
        public void run() {
            try {
                while (connected.get()) {
                    int frameLength = in.readInt();
                    byte[] frameData = in.readNBytes(frameLength);
                    var packet = FlatBufferUtils.FlatBufferWrapper
                            .fromByteArray(frameData)
                            .getOrThrow();
                    handlePacket(packet);
                }
            } catch (Exception e) {
                logger.error("Reader error", e);
            } finally {
                disconnect();
            }
        }
    }
}
