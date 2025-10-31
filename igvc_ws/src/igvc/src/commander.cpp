#include "igvc/node.hpp"

// messages
#include "std_msgs/msg/string.hpp"
#include "igvc_messages/msg/igvc_device_init.hpp"
#include "igvc_messages/srv/update_configuration.hpp"

class IGVCCommander : public IGVC::Node
{
public:
    IGVCCommander() : IGVC::Node("igvc_commander", true)
    {
        using namespace std::chrono_literals;
        mTimer = this->create_wall_timer(
            500ms,
            std::bind(&IGVCCommander::onTick, this)
        );

        // publisher and subscribers
        mDeviceInitPublisher = this->create_publisher<igvc_messages::msg::IGVCDeviceInit>(IGVC::Topics::DEVICE_INIT, 10);
        mConfigUpdatePublisher = this->create_publisher<std_msgs::msg::String>(IGVC::Topics::CONFIGURATION, 10);

        // services
        mUpdateConfigService = this->create_service<igvc_messages::srv::UpdateConfiguration>(
            IGVC::Services::UPDATE_CONFIGURATION,
            std::bind(&IGVCCommander::onUpdateConfiguration, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

    void init() override
    {
        // commander doesn't need to worry about this
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

    void onTick()
    {
        // get all nodes in the ros network
        std::vector<std::string> ros_nodes = this->get_node_names();

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

        IGVC::DeviceInitPayload payloadData {
            .system_state = getSystemState(),
            .device_state = IGVC::DeviceState::INITIALIZING,
            .configuration = getConfigurationJson(),
        };
        nlohmann::json payload = payloadData;

        igvc_messages::msg::IGVCDeviceInit message;
        message.target = name;
        message.json = payload.dump();
        mDeviceInitPublisher->publish(message);

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

    // services
    rclcpp::Service<igvc_messages::srv::UpdateConfiguration>::SharedPtr mUpdateConfigService;
};

int main(int argc, char *argv[])
{
    IGVC::Node::create_and_run<IGVCCommander>(argc, argv);
}