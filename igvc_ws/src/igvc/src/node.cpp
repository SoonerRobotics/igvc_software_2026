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

        mSystemContextSubscription = this->create_subscription<igvc_messages::msg::IGVCSystemContext>(
            IGVC::Topics::SYSTEM_CONTEXT,
            10,
            std::bind(&IGVC::Node::onSystemContextChanged, this, _1)
        );

        mSafetyLightsPublisher = this->create_publisher<igvc_messages::msg::SafetyLights>(
            IGVC::Topics::SAFETY_LIGHTS,
            10
        );

        // services
        mConfigUpdateClient = this->create_client<igvc_messages::srv::UpdateConfiguration>(
            IGVC::Services::UPDATE_CONFIGURATION
        );

        mDeviceStateUpdateClient = this->create_client<igvc_messages::srv::UpdateDeviceState>(
            IGVC::Services::UPDATE_DEVICE_STATE
        );

        mSystemContextUpdateClient = this->create_client<igvc_messages::srv::UpdateSystemContext>(
            IGVC::Services::UPDATE_SYSTEM_CONTEXT
        );
    }

    Node::~Node()
    {
    }

    SystemState Node::getSystemState()
    {
        return mSystemContext.state;
    }

    SystemContext Node::getSystemContext()
    {
        return mSystemContext;
    }

    bool Node::isMobilityEnabled()
    {
        return mSystemContext.isMobilityEnabled;
    }

    bool Node::isEmergencyStopped()
    {
        return mSystemContext.isEmergencyStopped;
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
        mSystemContext.state = state;
        setSystemContext(mSystemContext);
    }

    void Node::setMobilityEnabled(const bool enabled)
    {
        mSystemContext.isMobilityEnabled = enabled;
        setSystemContext(mSystemContext);
    }

    void Node::setEmergencyStopped(const bool estop)
    {
        mSystemContext.isEmergencyStopped = estop;
        setSystemContext(mSystemContext);
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

    void Node::setSafetyLights(const IGVC::Units::Color &color, const IGVC::SafetyLightsMode mode, const uint32_t blinkRateMs)
    {
        igvc_messages::msg::SafetyLights msg;
        msg.red = color.r();
        msg.green = color.g();
        msg.blue = color.b();
        msg.mode = static_cast<uint8_t>(mode);
        msg.blink_rate = blinkRateMs;
        mSafetyLightsPublisher->publish(msg);
    }

    void Node::setSystemContext(const SystemContext &context)
    {
        if (mIsCommander)
        {
            mSystemContext = context;
            return;
        }

        if (!mSystemContextUpdateClient->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "System state update service not available");
            return;
        }
        
        igvc_messages::srv::UpdateSystemContext::Request request;
        request.state = static_cast<int>(context.state);
        request.estop = context.isEmergencyStopped;
        request.mobility = context.isMobilityEnabled;
        auto future = mSystemContextUpdateClient->async_send_request(
            std::make_shared<igvc_messages::srv::UpdateSystemContext::Request>(request)
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
        mSystemContext = j["context"].get<SystemContext>();
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
        onConfigurationUpdated(mConfig);
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

    void Node::onSystemContextChanged(const igvc_messages::msg::IGVCSystemContext::SharedPtr msg)
    {
        SystemContext oldContext = mSystemContext;

        mSystemContext.isEmergencyStopped = msg->estop;
        mSystemContext.isMobilityEnabled = msg->mobility;
        mSystemContext.state = static_cast<SystemState>(msg->state);

        if (oldContext.state != mSystemContext.state)
        {
            onSystemStateUpdated(oldContext.state, mSystemContext.state);
        }

        if (oldContext.isMobilityEnabled != mSystemContext.isMobilityEnabled)
        {
            onMobilityUpdated(oldContext.isMobilityEnabled, mSystemContext.isMobilityEnabled);
        }

        if (oldContext.isEmergencyStopped != mSystemContext.isEmergencyStopped)
        {
            onEmergencyStoppedUpdated(oldContext.isEmergencyStopped, mSystemContext.isEmergencyStopped);
        }
    }
}