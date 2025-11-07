#include "igvc/node.hpp"

// messages
#include "std_msgs/msg/string.hpp"
#include "igvc_messages/msg/igvc_device_init.hpp"
#include "igvc_messages/msg/igvc_device_state.hpp"
#include "igvc_messages/msg/igvc_system_context.hpp"
#include "igvc_messages/srv/update_configuration.hpp"
#include "igvc_messages/srv/update_device_state.hpp"
#include "igvc_messages/srv/update_system_context.hpp"

class IGVCCommander : public IGVC::Node
{
public:
    IGVCCommander() : IGVC::Node("igvc_commander", true)
    {
        using namespace std::chrono_literals;
        mTimer = this->create_wall_timer(
            1000ms,
            std::bind(&IGVCCommander::onTick, this)
        );

        // publisher and subscribers
        mDeviceInitPublisher = this->create_publisher<igvc_messages::msg::IGVCDeviceInit>(IGVC::Topics::DEVICE_INIT, 10);
        mConfigUpdatePublisher = this->create_publisher<std_msgs::msg::String>(IGVC::Topics::CONFIGURATION, 10);
        mSystemContextPublisher = this->create_publisher<igvc_messages::msg::IGVCSystemContext>(IGVC::Topics::SYSTEM_CONTEXT, 10);
        mDeviceStatePublisher = this->create_publisher<igvc_messages::msg::IGVCDeviceState>(IGVC::Topics::DEVICE_STATE, 10);

        // services
        mUpdateConfigService = this->create_service<igvc_messages::srv::UpdateConfiguration>(
            IGVC::Services::UPDATE_CONFIGURATION,
            std::bind(&IGVCCommander::onUpdateConfiguration, this, std::placeholders::_1, std::placeholders::_2)
        );

        mUpdateDeviceStateService = this->create_service<igvc_messages::srv::UpdateDeviceState>(
            IGVC::Services::UPDATE_DEVICE_STATE,
            std::bind(&IGVCCommander::onUpdateDeviceState, this, std::placeholders::_1, std::placeholders::_2)
        );

        mUpdateSystemContextService = this->create_service<igvc_messages::srv::UpdateSystemContext>(
            IGVC::Services::UPDATE_SYSTEM_CONTEXT,
            std::bind(&IGVCCommander::onUpdateSystemContext, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

    void init() override
    {
        setDeviceState(IGVC::DeviceState::OPERATING);
    }

    void onEmergencyStoppedUpdated(bool oldEstop, bool newEstop) override
    {
        if (getSystemState() != IGVC::SystemState::DISABLED && newEstop == true)
        {
            RCLCPP_WARN(this->get_logger(), "Emergency stop detected, disabling system");
            setSystemState(IGVC::SystemState::DISABLED);
        }
    }
private:
    void onUpdateConfiguration(
        const std::shared_ptr<igvc_messages::srv::UpdateConfiguration::Request> request,
        std::shared_ptr<igvc_messages::srv::UpdateConfiguration::Response> response
    )
    {
        mConfig.loadFromJson(request->json);

        // publish new configuration
        std_msgs::msg::String msg;
        msg.data = request->json;
        mConfigUpdatePublisher->publish(msg);

        response->ok = true;
    }

    void onUpdateDeviceState(
        const std::shared_ptr<igvc_messages::srv::UpdateDeviceState::Request> request,
        std::shared_ptr<igvc_messages::srv::UpdateDeviceState::Response> response
    )
    {
        mDeviceStates[request->device] = static_cast<IGVC::DeviceState>(request->state);

        // publish that change
        igvc_messages::msg::IGVCDeviceState stateMsg;
        stateMsg.device = request->device;
        stateMsg.state = request->state;
        mDeviceStatePublisher->publish(stateMsg);

        response->ok = true;
    }

    void onUpdateSystemContext(
        const std::shared_ptr<igvc_messages::srv::UpdateSystemContext::Request> request,
        std::shared_ptr<igvc_messages::srv::UpdateSystemContext::Response> response
    )
    {
        // update our local state
        setSystemState(static_cast<IGVC::SystemState>(request->state));
        setMobilityEnabled(request->mobility);
        setEmergencyStopped(request->estop);
        
        // publish that change to everyone
        igvc_messages::msg::IGVCSystemContext stateMsg;
        stateMsg.state = request->state;
        stateMsg.mobility = request->mobility;
        stateMsg.estop = request->estop;
        mSystemContextPublisher->publish(stateMsg);

        response->ok = true;
    }

    void onTick()
    {
        // get all nodes in the ros network
        std::vector<std::string> ros_nodes = this->get_node_names();

        // print all nodes
        RCLCPP_DEBUG(this->get_logger(), "Discovered ROS Nodes:");
        for (const std::string &name : ros_nodes)
        {
            RCLCPP_DEBUG(this->get_logger(), " - %s", name.c_str());
        }

        // determine what nodes have died
        for (const std::string &name : mTrackedNodes)
        {
            bool exists = std::find(ros_nodes.begin(), ros_nodes.end(), name) != ros_nodes.end();
            if (!exists) continue;

            onNodeLost(name);
        }

        // determine what nodes are new
        for (const std::string &name : ros_nodes)
        {
            bool exists = std::find(mTrackedNodes.begin(), mTrackedNodes.end(), name.substr(1, name.size() - 1)) != mTrackedNodes.end();
            if (exists) continue;

            onNodeDiscovered(name.substr(1, name.size() - 1));
        }
    }

    void onNodeLost(const std::string &name)
    {
        RCLCPP_WARN(this->get_logger(), "Node %s lost", name.c_str());
    }

    void onNodeDiscovered(const std::string &name)
    {
        // a list of ignored nodes
        if (name == this->get_name()) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Node %s discovered, sending init payload", name.c_str());

        IGVC::DeviceInitPayload payloadData {
            .context = getSystemContext(),
            .device_state = IGVC::DeviceState::INITIALIZING,
            .configuration = getConfigurationJson(),
            .device_states = getAllDeviceStates()
        };
        nlohmann::json payload = payloadData;

        igvc_messages::msg::IGVCDeviceInit message;
        message.target = name;
        message.json = payload.dump();
        mDeviceInitPublisher->publish(message);

        // publish their device state
        igvc_messages::msg::IGVCDeviceState stateMsg;
        stateMsg.device = name;
        stateMsg.state = IGVC::DeviceState::INITIALIZING;
        mDeviceStatePublisher->publish(stateMsg);

        // add to found nodes
        mTrackedNodes.push_back(name);

        RCLCPP_INFO(this->get_logger(), "Node %s discovered", name.c_str());
    }

private:
    std::vector<std::string> mTrackedNodes;
    rclcpp::TimerBase::SharedPtr mTimer;

    // publishers
    rclcpp::Publisher<igvc_messages::msg::IGVCDeviceInit>::SharedPtr mDeviceInitPublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mConfigUpdatePublisher;
    rclcpp::Publisher<igvc_messages::msg::IGVCSystemContext>::SharedPtr mSystemContextPublisher;
    rclcpp::Publisher<igvc_messages::msg::IGVCDeviceState>::SharedPtr mDeviceStatePublisher;

    // services
    rclcpp::Service<igvc_messages::srv::UpdateConfiguration>::SharedPtr mUpdateConfigService;
    rclcpp::Service<igvc_messages::srv::UpdateDeviceState>::SharedPtr mUpdateDeviceStateService;
    rclcpp::Service<igvc_messages::srv::UpdateSystemContext>::SharedPtr mUpdateSystemContextService;
};

int main(int argc, char *argv[])
{
    IGVC::Node::create_and_run<IGVCCommander>(argc, argv);
}