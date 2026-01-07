package com.soonerrobotics.igvc;

import java.util.ArrayList;

import org.greenrobot.eventbus.EventBus;

import com.soonerrobotics.Constants;
import com.soonerrobotics.igvc.nodes.ImageFilteringNode;
import com.soonerrobotics.igvc.nodes.Node;
import com.soonerrobotics.igvc.services.ArcService;
import com.soonerrobotics.igvc.services.RobotSimulatorService;
import com.soonerrobotics.sus.BaseRobot;

public class Robot extends BaseRobot {

    private RobotSimulatorService _mSimulatorLink;
    private ArcService _mArcService;

    private final ArrayList<Node> _mNodes = new ArrayList<>();

    public Robot() {
        super(Constants.ROBOT_IDENTIFIER, Constants.SimulationConstants.IS_SIMULATION, Constants.ArcConstants.ENABLED);
    }

    @Override
    public void init() {
        _mNodes.add(new ImageFilteringNode());

        // Set up the event bus on our nodes
        for (Node node : _mNodes) {
            EventBus.getDefault().register(node);
        }

        // If we are in simulation, set up the simulator link
        if (isSimulation()) {
            _mSimulatorLink = new RobotSimulatorService(Constants.SimulationConstants.SIMULATION_ADDRESS,
                    Constants.SimulationConstants.SIMULATION_PORT);
            _mSimulatorLink.start();
        }

        // If we support web sockets, set up the web socket service
        if (supportsWebsocket()) {
            _mArcService = ArcService.getOrCreateInstance();
            _mArcService.start();
            System.out.println("ArcService started");

            // Add it to the event bus
            EventBus.getDefault().register(_mArcService);
        }
    }

    @Override
    public void shutdown() throws Exception {
        // Shutdown the nodes
        for (Node node : _mNodes) {
            EventBus.getDefault().unregister(node);
        }

        // Shutdown the simulator link
        if (_mSimulatorLink != null) {
            _mSimulatorLink.disconnect();
            EventBus.getDefault().unregister(_mSimulatorLink);
        }

        // Shutdown the arc service
        if (_mArcService != null) {
            _mArcService.stop();
            EventBus.getDefault().unregister(_mArcService);
        }
    }
}
