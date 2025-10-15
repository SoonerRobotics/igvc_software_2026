#pragma once

#include "rclcpp/rclcpp.hpp"

#include "igvc/defs.hpp"
#include "igvc_messages/msg/igvc_device_init.hpp"

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

        virtual void init() = 0;

    public:
        template <typename TNODE>
        static std::shared_ptr<TNODE> create_and_run(int argc, char *argv[])
        {
            rclcpp::init(argc, argv);

            std::shared_ptr<TNODE> node = std::make_shared<TNODE>();
            rclcpp::spin(node);

            rclcpp::shutdown();

            return node;
        }

    private:
        void onDeviceInitialized(const igvc_messages::msg::IGVCDeviceInit::SharedPtr msg);

    private:
        bool mIsCommander;
        SystemState mSystemState;
        DeviceState mDeviceState;

        rclcpp::Subscription<igvc_messages::msg::IGVCDeviceInit>::SharedPtr mDeviceInitSubscription;
    };
}