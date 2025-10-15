#include "igvc/node.hpp"
#include "igvc/json.hpp"
#include "igvc_messages/msg/igvc_device_init.hpp"

using std::placeholders::_1;

namespace IGVC
{
    Node::Node(const std::string &name, const bool isCommander) : rclcpp::Node(name)
    {
        mIsCommander = isCommander;

        mDeviceInitSubscription = this->create_subscription<igvc_messages::msg::IGVCDeviceInit>(
            "/igvc/device_init", 
            10,
            std::bind(&IGVC::Node::onDeviceInitialized, this, _1)
        );
    }

    Node::~Node()
    {
    }

    SystemState Node::getSystemState()
    {
        return mSystemState;
    }

    DeviceState Node::getDeviceState()
    {
        return mDeviceState;
    }

    void Node::setSystemState(const SystemState state)
    {

    }

    void Node::setDeviceState(const DeviceState state)
    {

    }

    void Node::onDeviceInitialized(const igvc_messages::msg::IGVCDeviceInit::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "received json: %s | %s", msg->target.c_str(), msg->json.c_str());
        // ignore messages for anything that is not us
        if (msg->target != this->get_name())
        {
            return;
        }

        // nlohmann::json json (msg->json);
        // auto payload = json.get<IGVC::DeviceInitPayload>();

        // // start
        // init();

        RCLCPP_INFO(this->get_logger(), "Received initialization packet");
    }
}