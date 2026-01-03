package com.soonerrobotics.arc.messages;

import java.nio.ByteBuffer;

public abstract class Message<T> {
    public abstract void write(final ByteBuffer buffer);
    public abstract T read(final ByteBuffer buffer);
}
