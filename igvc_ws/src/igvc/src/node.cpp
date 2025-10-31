#include "igvc/node.hpp"
#include "igvc/json.hpp"
#include "igvc_messages/msg/igvc_device_init.hpp"

using std::placeholders::_1;

namespace IGVC
{
    Node::Node(const std::string &name, const bool isCommander)
        : rclcpp::Node(name), mIsCommander(isCommander), mConfig(Config::getInstance(
            std::bind(&IGVC::Node::onLocalConfigUpdated, this)
        ))
    {
        mDeviceInitSubscription = this->create_subscription<igvc_messages::msg::IGVCDeviceInit>(
            IGVC::Topics::DEVICE_INIT,
            10,
            std::bind(&IGVC::Node::onDeviceInitialized, this, _1)
        );

        mConfigUpdateSubscription = this->create_subscription<std_msgs::msg::String>(
            IGVC::Topics::CONFIGURATION,
            10,
            std::bind(&IGVC::Node::onConfigUpdated, this, _1)
        );

        mConfigUpdateClient = this->create_client<igvc_messages::srv::UpdateConfiguration>(
            IGVC::Services::UPDATE_CONFIGURATION
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

    std::string Node::getConfigurationJson()
    {
        nlohmann::json j = mConfig;
        return j.dump();
    }

    void Node::setSystemState(const SystemState state)
    {

    }

    void Node::setDeviceState(const DeviceState state)
    {

    }

    void Node::onDeviceInitialized(const igvc_messages::msg::IGVCDeviceInit::SharedPtr msg)
    {
        // ignore messages for anything that is not us
        if (msg->target != std::string(this->get_name()))
        {
            return;
        }

        nlohmann::json j = nlohmann::json::parse(msg->json);
        mSystemState = static_cast<SystemState>(j["system_state"].get<int>());
        mDeviceState = static_cast<DeviceState>(j["device_state"].get<int>());
        mConfig.loadFromJson(j["configuration"].get<std::string>());
        
        // start
        init();
    }

    void Node::onConfigUpdated(const std_msgs::msg::String::SharedPtr msg)
    {
        mConfig.loadFromJson(msg->data);
    }

    void Node::onLocalConfigUpdated()
    {
        if (!mConfigUpdateClient->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Configuration update service not available");
            return;
        }

        auto request = std::make_shared<igvc_messages::srv::UpdateConfiguration::Request>();
        request->json = getConfigurationJson();
        auto future = mConfigUpdateClient->async_send_request(request);

        // wait for the result
        // try
        // {
        //     auto response = future.get();
        //     RCLCPP_INFO(this->get_logger(), "Configuration update response: %s", response->ok ? "ok" : "failed");
        // }
        // catch (const std::exception &e)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to call configuration update service: %s", e.what());
        // }
    }
}