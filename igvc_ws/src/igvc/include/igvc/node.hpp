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
#include "igvc_messages/msg/igvc_device_init.hpp"
#include "igvc_messages/srv/update_configuration.hpp"

namespace IGVC
{
    class Node : public rclcpp::Node
    {
    public:
        Node(const std::string &name, const bool isCommander = false);
        ~Node();
        
    protected:
        SystemState getSystemState();
        DeviceState getDeviceState();

        void setDeviceState(const DeviceState state);
        void setSystemState(const SystemState state);
        std::string getConfigurationJson();

        virtual void init() = 0;

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

    protected:
        Config &mConfig;

    private:
        bool mIsCommander;
        SystemState mSystemState;
        DeviceState mDeviceState;

        rclcpp::Subscription<igvc_messages::msg::IGVCDeviceInit>::SharedPtr mDeviceInitSubscription;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mConfigUpdateSubscription;
        rclcpp::Client<igvc_messages::srv::UpdateConfiguration>::SharedPtr mConfigUpdateClient;
    };
}