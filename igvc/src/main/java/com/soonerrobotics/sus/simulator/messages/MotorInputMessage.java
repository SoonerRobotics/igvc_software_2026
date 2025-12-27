package com.soonerrobotics.sus.simulator.messages;

import java.io.DataOutputStream;
import java.io.IOException;

public class MotorInputMessage implements Message<MotorInputMessage> {
    private static final MessageType MESSAGE_ID = MessageType.MOTORINPUT_2026;
    
    public double forwardVelocity;
    public double sidewaysVelocity;
    public double angularVelocity;

    public MotorInputMessage(double forwardVelocity, double sidewaysVelocity, double angularVelocity) {
        this.forwardVelocity = forwardVelocity;
        this.sidewaysVelocity = sidewaysVelocity;
        this.angularVelocity = angularVelocity;
    }

    @Override
    public void write(DataOutputStream output) throws IOException {
        output.writeInt(MESSAGE_ID.getValue());
        output.writeDouble(forwardVelocity);
        output.writeDouble(sidewaysVelocity);
        output.writeDouble(angularVelocity);
    }
}
