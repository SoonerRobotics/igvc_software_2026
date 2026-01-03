package com.soonerrobotics.sus;

public abstract class BaseRobot {
    private final String _mIdentifier;
    private final boolean _mIsSimulation;
    private final boolean _mSupportsWebsocket;

    public BaseRobot(String identifier, boolean isSimulation, boolean supportsWebsocket) {
        _mIsSimulation = isSimulation;
        _mSupportsWebsocket = supportsWebsocket;
        _mIdentifier = identifier;
    }

    public static void startRobot(BaseRobot robot)
    {
        robot.init();
    }
    
    // Getters

    public String getIdentifier() {
        return _mIdentifier;
    }

    public boolean isSimulation() {
        return _mIsSimulation;
    }

    public boolean supportsWebsocket() {
        return _mSupportsWebsocket;
    }

    public abstract void init();
    public abstract void shutdown() throws Exception;
}
