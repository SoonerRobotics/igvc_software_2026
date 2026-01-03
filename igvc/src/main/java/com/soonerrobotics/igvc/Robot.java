package com.soonerrobotics.igvc;

import java.util.ArrayList;

import org.greenrobot.eventbus.EventBus;

import com.soonerrobotics.arc.ArcService;
import com.soonerrobotics.igvc.constants.ArcConstants;
import com.soonerrobotics.igvc.constants.GeneralConstants;
import com.soonerrobotics.igvc.constants.SimulationConstants;
import com.soonerrobotics.igvc.nodes.ImageFilteringNode;
import com.soonerrobotics.igvc.nodes.Node;
import com.soonerrobotics.igvc.services.RobotSimulatorService;
import com.soonerrobotics.sus.BaseRobot;

public class Robot extends BaseRobot {

    private RobotSimulatorService _mSimulatorLink;
    private ArcService _mArcService;

    private final ArrayList<Node> _mNodes = new ArrayList<>();

    public Robot() {
        super(GeneralConstants.ROBOT_IDENTIFIER, SimulationConstants.IS_SIMULATION, ArcConstants.ENABLED);
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
            _mSimulatorLink = new RobotSimulatorService(SimulationConstants.SIMULATION_ADDRESS,
                    SimulationConstants.SIMULATION_PORT);
            _mSimulatorLink.start();
        }

        // If we support web sockets, set up the web socket service
        if (supportsWebsocket()) {
            _mArcService = ArcService.getOrCreateInstance();
            _mArcService.start();

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
            EventBus.getDefault().unregister(_mArcService);
            _mArcService.stop();
        }
    }
}
