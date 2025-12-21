package com.soonerrobotics.sus;

public abstract class BaseRobot {
    private final String mIdentifier;
    protected boolean mIsSimulation;

    public BaseRobot(String identifier) {
        mIdentifier = identifier;
    }

    public static void startRobot(BaseRobot robot)
    {
        
    }
    
    // Getters

    public String getIdentifier() {
        return mIdentifier;
    }

    public boolean isSimulation() {
        return mIsSimulation;
    }

    public abstract void init();
}
