package com.soonerrobotics.sus.simulator.messages;

public enum MessageType {
    CONNECT(1),
    UNUSED(999),
    MOTORINPUT_2026(20260);

    private final int value;

    MessageType(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }

    public static MessageType fromInt(int value) {
        for (MessageType type : MessageType.values()) {
            if (type.getValue() == value) {
                return type;
            }
        }
        throw new IllegalArgumentException("Unknown MessageType value: " + value);
    }
}
