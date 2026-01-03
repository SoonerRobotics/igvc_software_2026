package com.soonerrobotics.sus.simulator.packets;

import java.io.DataOutputStream;
import java.io.IOException;

public class MotorInputPacket implements Packet<MotorInputPacket> {
    private static final PacketType PACKET_ID = PacketType.S_MOTORINPUT_2026;
    
    public double forwardVelocity;
    public double sidewaysVelocity;
    public double angularVelocity;

    public MotorInputPacket(double forwardVelocity, double sidewaysVelocity, double angularVelocity) {
        this.forwardVelocity = forwardVelocity;
        this.sidewaysVelocity = sidewaysVelocity;
        this.angularVelocity = angularVelocity;
    }

    @Override
    public void write(DataOutputStream output) throws IOException {
        output.writeInt(PACKET_ID.getValue());
        output.writeDouble(forwardVelocity);
        output.writeDouble(sidewaysVelocity);
        output.writeDouble(angularVelocity);
    }
}
