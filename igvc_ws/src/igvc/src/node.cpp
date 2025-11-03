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
        // pubsub
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

        mDeviceStateSubscription = this->create_subscription<igvc_messages::msg::IGVCDeviceState>(
            IGVC::Topics::DEVICE_STATE,
            10,
            std::bind(&IGVC::Node::onDeviceStateChanged, this, _1)
        );

        mSystemStateSubscription = this->create_subscription<igvc_messages::msg::IGVCSystemState>(
            IGVC::Topics::SYSTEM_STATE,
            10,
            std::bind(&IGVC::Node::onSystemStateChanged, this, _1)
        );

        // services
        mConfigUpdateClient = this->create_client<igvc_messages::srv::UpdateConfiguration>(
            IGVC::Services::UPDATE_CONFIGURATION
        );

        mDeviceStateUpdateClient = this->create_client<igvc_messages::srv::UpdateDeviceState>(
            IGVC::Services::UPDATE_DEVICE_STATE
        );

        mSystemStateUpdateClient = this->create_client<igvc_messages::srv::UpdateSystemState>(
            IGVC::Services::UPDATE_SYSTEM_STATE
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

    DeviceState Node::getDeviceState(const std::string &deviceName)
    {
        if (mDeviceStates.find(deviceName) != mDeviceStates.end())
        {
            return mDeviceStates[deviceName];
        }
        return DeviceState::UNKNOWN;
    }

    std::map<std::string, DeviceState> Node::getAllDeviceStates()
    {
        return mDeviceStates;
    }

    std::string Node::getConfigurationJson()
    {
        nlohmann::json j = mConfig;
        return j.dump();
    }

    void Node::setSystemState(const SystemState state)
    {
        if (mIsCommander)
        {
            mSystemState = state;
            return;
        }

        if (!mSystemStateUpdateClient->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "System state update service not available");
            return;
        }
        
        igvc_messages::srv::UpdateSystemState::Request request;
        request.state = static_cast<int>(state);
        auto future = mSystemStateUpdateClient->async_send_request(
            std::make_shared<igvc_messages::srv::UpdateSystemState::Request>(request)
        );
    }

    void Node::setDeviceState(const DeviceState state)
    {
        if (mIsCommander)
        {
            mDeviceState = state;
            return;
        }

        if (!mDeviceStateUpdateClient->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Device state update service not available");
            return;
        }

        igvc_messages::srv::UpdateDeviceState::Request request;
        request.state = static_cast<int>(state);
        request.device = this->get_name();
        auto future = mDeviceStateUpdateClient->async_send_request(
            std::make_shared<igvc_messages::srv::UpdateDeviceState::Request>(request)
        );
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

        // loop through device states map and apply locally
        for (auto& [deviceName, stateValue] : j["device_states"].items())
        {
            mDeviceStates[deviceName] = static_cast<DeviceState>(stateValue.get<int>());
        }
        
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

    void Node::onDeviceStateChanged(const igvc_messages::msg::IGVCDeviceState::SharedPtr msg)
    {
        mDeviceStates[msg->device] = static_cast<DeviceState>(msg->state);
        if (msg->device == this->get_name())
        {
            mDeviceState = static_cast<DeviceState>(msg->state);
        }
    }

    void Node::onSystemStateChanged(const igvc_messages::msg::IGVCSystemState::SharedPtr msg)
    {
        mSystemState = static_cast<SystemState>(msg->state);
    }
}