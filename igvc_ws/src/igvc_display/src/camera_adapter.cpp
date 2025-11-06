#include "igvc/node.hpp"
#include "igvc/utilities.hpp"
#include "igvc/json.hpp"
#include "igvc_display/packets.hpp"
#include "igvc_display/limiter.hpp"

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <vector>
#include <cstdint>
#include <iostream>

class IGVCCameraAdapter : public IGVC::Node
{
public:
    IGVCCameraAdapter() : IGVC::Node("igvc_camera_adapter")
    {
    }

    void init() override
    {
        setDeviceState(IGVC::DeviceState::READY);
    }
};

int main(int argc, char *argv[])
{
    IGVC::Node::create_and_run<IGVCCameraAdapter>(argc, argv);
}