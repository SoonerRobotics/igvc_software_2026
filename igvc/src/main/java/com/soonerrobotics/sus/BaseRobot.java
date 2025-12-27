package com.soonerrobotics.sus;

import com.soonerrobotics.constants.SimulationConstants;

public abstract class BaseRobot {
    private final String mIdentifier;
    protected boolean mIsSimulation = SimulationConstants.IS_SIMULATION;

    public BaseRobot(String identifier) {
        mIdentifier = identifier;
    }

    public static void startRobot(BaseRobot robot)
    {
        robot.init();
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
