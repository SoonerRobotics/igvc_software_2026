package com.soonerrobotics.sus.simulator.messages;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

public interface Message<T> {
    void write(DataOutputStream output) throws IOException;

    default T read(DataInputStream input) throws IOException {
        return null;
    }
}
