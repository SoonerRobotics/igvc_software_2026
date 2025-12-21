package com.soonerrobotics;

import com.soonerrobotics.constants.GeneralConstants;
import com.soonerrobotics.constants.SimulationConstants;
import com.soonerrobotics.sus.BaseRobot;

public class Robot extends BaseRobot {
    public Robot() {
        super(GeneralConstants.ROBOT_IDENTIFIER);
    }

    private RobotSimulatorLink mSimulatorLink;

    @Override
    public void init() {
        // If we are in simulation, set up the simulator link
        if (isSimulation()) {
            mSimulatorLink = new RobotSimulatorLink(SimulationConstants.SIMULATION_ADDRESS, SimulationConstants.SIMULATION_PORT);

            // TODO: I think this needs to be on its own thread
            mSimulatorLink.connect();
        }
    }
}
