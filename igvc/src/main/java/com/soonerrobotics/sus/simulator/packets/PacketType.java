package com.soonerrobotics.sus.simulator.packets;

/**
 * Enum representing different types of messages exchanged with the simulator.
 * 
 * Naming convention: DIRECTION _ MESSAGENAME
 * DIRECTION: S/R/SR (Send/Receive/SendReceive)
 */
public enum PacketType {
    UNUSED(999),

    S_MOTORINPUT_2026(20260),
    R_MOTORFEEDBACK_2026(20261),
    R_CAMERAFRAME_2026(20262),;

    private final int value;

    PacketType(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }

    public static PacketType fromInt(int value) {
        for (PacketType type : PacketType.values()) {
            if (type.getValue() == value) {
                return type;
            }
        }
        throw new IllegalArgumentException("Unknown MessageType value: " + value);
    }
}
