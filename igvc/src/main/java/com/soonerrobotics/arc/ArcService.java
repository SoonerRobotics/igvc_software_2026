package com.soonerrobotics.arc;

import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.greenrobot.eventbus.Subscribe;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import com.soonerrobotics.igvc.constants.ArcConstants;
import com.soonerrobotics.igvc.events.ImageEvent;

@SuppressWarnings("unused")
public class ArcService extends WebSocketServer {
    private static final Logger logger = LogManager.getLogger(ArcService.class);

    private static ArcService _mInstance = null;

    public static ArcService getOrCreateInstance() {
        if (_mInstance == null) {
            _mInstance = new ArcService(ArcConstants.PORT);
        }

        return _mInstance;
    }

    private final Set<WebSocket> _mClients = Collections.synchronizedSet(new HashSet<>());

    protected ArcService(int port) {
        super(new InetSocketAddress(port));
    }

    // region WebSocketServer Methods

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        logger.info("New WebSocket connection from " + conn.getRemoteSocketAddress());
        _mClients.add(conn);
        conn.send("Hello from IGVC ArcService!");
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
    public void onImageReceived(ImageEvent event) {
        ByteBuffer imageData = ArcTransformers.transformImageEvent(event);

        synchronized (_mClients) {
            for (WebSocket client : _mClients) {
                client.send(imageData);
            }
        }
    }

    // endregion
}
