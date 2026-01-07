package com.soonerrobotics.igvc.services;

import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.time.Instant;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.core.LogEvent;
import org.greenrobot.eventbus.Subscribe;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import com.google.flatbuffers.FlatBufferBuilder;
import com.soonerrobotics.Constants;
import com.soonerrobotics.flatbuffers.ImageFrame;
import com.soonerrobotics.flatbuffers.arc.ArcLog;
import com.soonerrobotics.igvc.events.ImageFrameEvent;
import com.soonerrobotics.sus.utils.FlatBufferUtils;

@SuppressWarnings("unused")
public class ArcService extends WebSocketServer {
    private static final Logger logger = LogManager.getLogger(ArcService.class);

    private static ArcService _mInstance = null;

    public static ArcService getOrCreateInstance() {
        if (_mInstance == null) {
            _mInstance = new ArcService(Constants.ArcConstants.PORT);
        }

        return _mInstance;
    }

    private final Set<WebSocket> _mClients = Collections.synchronizedSet(new HashSet<>());

    protected ArcService(int port) {
        super(new InetSocketAddress(port));

        setReuseAddr(true);
    }

    // region WebSocketServer Methods

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        logger.info("New WebSocket connection from " + conn.getRemoteSocketAddress());
        _mClients.add(conn);
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        logger.info("WebSocket connection closed from " + conn.getRemoteSocketAddress());
        _mClients.remove(conn);
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        logger.error("WebSocket error", ex);
    }

    @Override
    public void onStart() {
        logger.info("WebSocket server started on port " + getPort());
    }

    @Override
    public void onMessage(WebSocket conn, String message) {
        logger.info("Received message from " + conn.getRemoteSocketAddress() + ": " + message);
    }

    // endregion

    // region Event Listeners

    @Subscribe
    public void onImageReceived(ImageFrameEvent event) {
        ImageFrame frame = event.frame();
        ByteBuffer imageDataBB = frame.getByteBuffer();
        byte[] imageData = new byte[imageDataBB.remaining()];
        imageDataBB.get(imageData);

        var wrapper = FlatBufferUtils.FlatBufferWrapper.create(FlatBufferUtils.FlatBufferType.IMAGE_FRAME, imageData);
        if (wrapper.isError())
        {
            logger.error("Failed to wrap ImageFrame for broadcasting to ARC: " + wrapper.getError());
            return;
        }
        
        broadcast(wrapper.get().toByteArray());
    }

    @Subscribe
    public void onLogEvent(LogEvent event) {
        FlatBufferBuilder builder = new FlatBufferBuilder(1024);
        int methodCallerOffset = builder.createString(event.getLoggerName());
        int logLevelOffset = builder.createString(event.getLevel().toString());
        int messageOffset = builder.createString(event.getMessage().getFormattedMessage());

        var arcLog = ArcLog.createArcLog(
            builder,
            Instant.now().toEpochMilli(),
            0, // TODO: Use a real sequence number
            methodCallerOffset,
            logLevelOffset,
            messageOffset
        );
        builder.finish(arcLog);
        // broadcast(builder.dataBuffer());
    }

    // endregion
}
