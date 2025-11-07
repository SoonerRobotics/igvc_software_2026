#pragma once

// general
#include <cstdint>
#include <csignal>

// ros
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// igvc
#include "igvc/defs.hpp"
#include "igvc/topics.hpp"
#include "igvc/config.hpp"
#include "igvc/utilities.hpp"
#include "igvc/units.hpp"
#include "igvc_messages/msg/igvc_device_init.hpp"
#include "igvc_messages/msg/igvc_device_state.hpp"
#include "igvc_messages/msg/igvc_system_context.hpp"
#include "igvc_messages/msg/safety_lights.hpp"
#include "igvc_messages/srv/update_configuration.hpp"
#include "igvc_messages/srv/update_device_state.hpp"
#include "igvc_messages/srv/update_system_context.hpp"

namespace IGVC
{
    class Node : public rclcpp::Node
    {
    public:
        Node(const std::string &name, const bool isCommander = false);
        ~Node();
        
    protected:
        SystemState getSystemState();
        SystemContext getSystemContext();
        bool isMobilityEnabled();
        bool isEmergencyStopped();
        DeviceState getDeviceState();
        DeviceState getDeviceState(const std::string &deviceName);
        std::map<std::string, DeviceState> getAllDeviceStates();

        void setDeviceState(const DeviceState state);
        void setSystemState(const SystemState state);
        void setMobilityEnabled(const bool enabled);
        void setEmergencyStopped(const bool estop);
        void setSafetyLights(const IGVC::Units::Color &color, const IGVC::SafetyLightsMode mode, const uint32_t blinkRateMs = 750);
        std::string getConfigurationJson();

        virtual void init() = 0;
        virtual void onSystemStateUpdated(SystemState oldState, SystemState newState) {}
        virtual void onMobilityUpdated(bool oldMobility, bool newMobility) {}
        virtual void onEmergencyStoppedUpdated(bool oldEstop, bool newEstop) {}
        virtual void onConfigurationUpdated(const Config &config) {}

    public:
        template <typename TNODE, typename... TARGS>
        static std::shared_ptr<TNODE> create_and_run(int argc, char *argv[], TARGS&&... args)
        {
            rclcpp::init(argc, argv);

            auto signalHandler = [](int signum)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Signal %d received, shutting down.", signum);
                rclcpp::shutdown();
            };
            std::signal(SIGINT, signalHandler);
            std::signal(SIGTERM, signalHandler);

            std::shared_ptr<TNODE> node = std::make_shared<TNODE>(std::forward<TARGS>(args)...);
            rclcpp::executors::MultiThreadedExecutor executor;
            executor.add_node(node);
            executor.spin();

            rclcpp::shutdown();
            return node;
        }

    private:
        void onDeviceInitialized(const igvc_messages::msg::IGVCDeviceInit::SharedPtr msg);
        void onConfigUpdated(const std_msgs::msg::String::SharedPtr msg);
        void onLocalConfigUpdated();
        void onDeviceStateChanged(const igvc_messages::msg::IGVCDeviceState::SharedPtr msg);
        void onSystemContextChanged(const igvc_messages::msg::IGVCSystemContext::SharedPtr msg);
        void setSystemContext(const SystemContext &context);

    protected:
        // local node state
        bool mIsCommander;
        Config &mConfig;
        
        // other stuff
        std::map<std::string, DeviceState> mDeviceStates;
        
    private:
        SystemContext mSystemContext;
        DeviceState mDeviceState;
                
        rclcpp::Subscription<igvc_messages::msg::IGVCDeviceInit>::SharedPtr mDeviceInitSubscription;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mConfigUpdateSubscription;
        rclcpp::Subscription<igvc_messages::msg::IGVCDeviceState>::SharedPtr mDeviceStateSubscription;
        rclcpp::Subscription<igvc_messages::msg::IGVCSystemContext>::SharedPtr mSystemContextSubscription;

        rclcpp::Publisher<igvc_messages::msg::SafetyLights>::SharedPtr mSafetyLightsPublisher;

        rclcpp::Client<igvc_messages::srv::UpdateConfiguration>::SharedPtr mConfigUpdateClient;
        rclcpp::Client<igvc_messages::srv::UpdateDeviceState>::SharedPtr mDeviceStateUpdateClient;
        rclcpp::Client<igvc_messages::srv::UpdateSystemContext>::SharedPtr mSystemContextUpdateClient;
    };
}