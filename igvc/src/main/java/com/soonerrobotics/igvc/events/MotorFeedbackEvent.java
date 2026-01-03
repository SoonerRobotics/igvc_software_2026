package com.soonerrobotics.igvc.events;

public class MotorFeedbackEvent {
    private final float deltaX;
    private final float deltaY;
    private final float deltaTheta;

    public MotorFeedbackEvent(float deltaX, float deltaY, float deltaTheta) {
        this.deltaX = deltaX;
        this.deltaY = deltaY;
        this.deltaTheta = deltaTheta;
    }

    public float getDeltaX() {
        return deltaX;
    }

    public float getDeltaY() {
        return deltaY;
    }

    public float getDeltaTheta() {
        return deltaTheta;
    }
}
