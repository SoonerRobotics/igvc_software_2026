package com.soonerrobotics.sus.simulator.messages;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

public class ConnectMessage implements Message<ConnectMessage> {
    private static final MessageType MESSAGE_ID = MessageType.CONNECT;

    private final String mClientIdentifier;

    public ConnectMessage(String clientIdentifier) {
        this.mClientIdentifier = clientIdentifier;
    }

    public String getmClientIdentifier() {
        return mClientIdentifier;
    }

    @Override
    public void write(DataOutputStream output) throws IOException {
        byte[] identifierBytes = mClientIdentifier.getBytes(StandardCharsets.UTF_8);

        output.writeInt(MESSAGE_ID.getValue());
        output.writeInt(identifierBytes.length);
        output.write(identifierBytes);
    }

    @Override
    public ConnectMessage read(DataInputStream input) throws IOException {
        int length = input.readInt();
        byte[] identifierBytes = new byte[length];
        input.readFully(identifierBytes);
        String iClientIdentifier = new String(identifierBytes, StandardCharsets.UTF_8);
        return new ConnectMessage(iClientIdentifier);
    }
}
