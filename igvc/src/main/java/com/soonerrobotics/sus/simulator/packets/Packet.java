package com.soonerrobotics.sus.simulator.packets;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

public interface Packet<T> {
    void write(DataOutputStream output) throws IOException;

    default T read(DataInputStream input) throws IOException {
        return null;
    }
}
