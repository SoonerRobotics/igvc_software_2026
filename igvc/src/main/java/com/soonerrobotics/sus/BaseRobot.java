package com.soonerrobotics.sus;

import java.util.concurrent.atomic.AtomicBoolean;

public abstract class BaseRobot {
    private final String _mIdentifier;
    private final boolean _mIsSimulation;
    private final boolean _mSupportsWebsocket;

    public static AtomicBoolean isShuttingDown = new AtomicBoolean(false);

    public BaseRobot(String identifier, boolean isSimulation, boolean supportsWebsocket) {
        _mIsSimulation = isSimulation;
        _mSupportsWebsocket = supportsWebsocket;
        _mIdentifier = identifier;
    }

    public static void startRobot(BaseRobot robot)
    {
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            try {
                robot.shutdown();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }));

        robot.UserInputShutdownListener();

        robot.init();
    }

    private void UserInputShutdownListener() {
        Thread userInputThread = new Thread(() -> {
            java.util.Scanner scanner = new java.util.Scanner(System.in);
            while (true) {
                String input = scanner.nextLine();
                if (input.equals(":shutdown")) {
                    try {
                        scanner.close();
                        isShuttingDown.set(true);
                        shutdown();
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                    break;
                }
            }
        });
        userInputThread.setDaemon(true);
        userInputThread.start();
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
