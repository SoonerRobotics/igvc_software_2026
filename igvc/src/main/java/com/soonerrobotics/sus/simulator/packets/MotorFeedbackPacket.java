package com.soonerrobotics.sus.simulator.packets;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

public class MotorFeedbackPacket implements Packet<MotorFeedbackPacket> {
    private static final PacketType PACKET_ID = PacketType.R_MOTORFEEDBACK_2026;
    
    public double deltaX;
    public double deltaY;
    public double deltaTheta;

    public MotorFeedbackPacket() {
        this(0.0, 0.0, 0.0);
    }

    public MotorFeedbackPacket(double deltaX, double deltaY, double deltaTheta) {
        this.deltaX = deltaX;
        this.deltaY = deltaY;
        this.deltaTheta = deltaTheta;
    }

    @Override
    public void write(DataOutputStream output) throws IOException {
        output.writeInt(PACKET_ID.getValue());
        output.writeDouble(deltaX);
        output.writeDouble(deltaY);
        output.writeDouble(deltaTheta);
    }

    @Override
    public MotorFeedbackPacket read(DataInputStream input) throws IOException {
        double _deltaX = input.readDouble();
        double _deltaY = input.readDouble();
        double _deltaTheta = input.readDouble();
        return new MotorFeedbackPacket(_deltaX, _deltaY, _deltaTheta);
    }
}
