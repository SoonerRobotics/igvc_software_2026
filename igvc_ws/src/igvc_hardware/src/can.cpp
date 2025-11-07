#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "igvc/node.hpp"
#include "igvc_hardware/can.hpp"

#include "igvc_messages/msg/motor_input.hpp"
#include "igvc_messages/msg/motor_feedback.hpp"
#include "igvc_messages/msg/safety_lights.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

using namespace std::chrono_literals;

class IGVCHardwareCanNode : public IGVC::Node
{
public:
    IGVCHardwareCanNode() : IGVC::Node("igvc_hardware_can") {}

    void init() override
    {
        mMotorInputSubscription = this->create_subscription<igvc_messages::msg::MotorInput>(
            IGVC::Topics::MOTOR_INPUT, 10,
            std::bind(&IGVCHardwareCanNode::onMotorInputReceived, this, std::placeholders::_1));

        mSafetyLightsSubscription = this->create_subscription<igvc_messages::msg::SafetyLights>(
            IGVC::Topics::SAFETY_LIGHTS, 10,
            std::bind(&IGVCHardwareCanNode::onSafetyLightsReceived, this, std::placeholders::_1));

        mMotorFeedbackPublisher = this->create_publisher<igvc_messages::msg::MotorFeedback>(
            IGVC::Topics::MOTOR_FEEDBACK, 10);

        setDeviceState(IGVC::DeviceState::READY);
        initCan("can0");

        can_reader_thread_ = std::thread(&IGVCHardwareCanNode::canReaderThread, this);
        can_writer_thread_ = std::thread(&IGVCHardwareCanNode::canWriterThread, this);
    }

private:
    void onMotorInputReceived(const igvc_messages::msg::MotorInput::SharedPtr msg)
    {
        // Our msg delivers the velocities as floats, but our CAN packet uses int16_t
        // so we need to convert them by multiplying by 1000 and casting to int16_t
        // this gives us a resolution of 0.001 m/s or rad/s

        IGVC::CAN::Packets::MotorInput packet;
        packet.forward_velocity = static_cast<int16_t>(msg->forward_velocity * 1000);
        packet.sideways_velocity = static_cast<int16_t>(msg->sideways_velocity * 1000);
        packet.angular_velocity = static_cast<int16_t>(msg->angular_velocity * 1000);
        struct can_frame frame;
        frame.can_id = IGVC::CAN::IDS::MOTOR_INPUT;
        frame.can_dlc = sizeof(IGVC::CAN::Packets::MotorInput);
        std::memcpy(frame.data, &packet, sizeof(IGVC::CAN::Packets::MotorInput));
        can_send_queue_.push(frame);
    }

    void onSafetyLightsReceived(const igvc_messages::msg::SafetyLights::SharedPtr msg)
    {
        IGVC::CAN::Packets::SafetyLights packet;
        packet.r = static_cast<uint8_t>(msg->red);
        packet.g = static_cast<uint8_t>(msg->green);
        packet.b = static_cast<uint8_t>(msg->blue);
        packet.mode = static_cast<uint8_t>(msg->mode);
        packet.blink_rate_ms = static_cast<uint16_t>(msg->blink_rate);
        struct can_frame frame;
        frame.can_id = IGVC::CAN::IDS::SAFETY_LIGHTS;
        frame.can_dlc = sizeof(IGVC::CAN::Packets::SafetyLights);
        std::memcpy(frame.data, &packet, sizeof(IGVC::CAN::Packets::SafetyLights));
        can_send_queue_.push(frame);
    }

private:
    void initCan(std::string device)
    {
        struct ifreq ifr;
        struct sockaddr_can addr;

        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while opening CAN socket");
            return;
        }

        std::strncpy(ifr.ifr_name, device.c_str(), IFNAMSIZ - 1);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error getting interface index for %s", device.c_str());
            return;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in socket bind for %s", device.c_str());
            return;
        }

        setDeviceState(IGVC::DeviceState::OPERATING);
        RCLCPP_INFO(this->get_logger(), "Successfully initialized CAN interface %s", device.c_str());
    }

    bool readFrame(struct can_frame &frame)
    {
        ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0)
        {
            RCLCPP_WARN(this->get_logger(), "CAN read error");
            return false;
        }

        if (nbytes < (ssize_t)sizeof(struct can_frame))
        {
            RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame");
            return false;
        }

        return true;
    }

    void sendFrame(const struct can_frame &frame)
    {
        ssize_t nbytes = write(can_socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0)
        {
            RCLCPP_WARN(this->get_logger(), "CAN write error");
        }
        else if (nbytes < (ssize_t)sizeof(struct can_frame))
        {
            RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame sent");
        }
    }

    void printFrame(const struct can_frame &frame)
    {
        std::ostringstream oss;
        oss << "CAN ID: " << std::hex << frame.can_id << " DLC: " << std::dec << (int)frame.can_dlc << " Data: ";
        for (int i = 0; i < frame.can_dlc; i++)
        {
            oss << std::hex << (int)frame.data[i] << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    void onCanReceived(const struct can_frame &frame)
    {
        switch (frame.can_id)
        {
        case IGVC::CAN::IDS::MOTOR_FEEDBACK:
        {
            IGVC::CAN::Packets::MotorFeedback packet;
            std::memcpy(&packet, frame.data, sizeof(IGVC::CAN::Packets::MotorFeedback));

            igvc_messages::msg::MotorFeedback msg;
            msg.delta_x = static_cast<float>(packet.delta_x) / 1000.0f;
            msg.delta_y = static_cast<float>(packet.delta_y) / 1000.0f;
            msg.delta_theta = static_cast<float>(packet.delta_theta) / 1000.0f;
            mMotorFeedbackPublisher->publish(msg);
            break;
        }
        
        case IGVC::CAN::IDS::ESTOP:
        {
            // TODO: Set estop
            RCLCPP_WARN(this->get_logger(), "ESTOP Received");
            break;
        }

        case IGVC::CAN::IDS::MOBILITY_START:
        {
            // TODO: Set mobility
            RCLCPP_INFO(this->get_logger(), "Mobility Start Received");
            break;
        }

        case IGVC::CAN::IDS::MOBILITY_STOP:
        {
            // TODO: Set mobility
            RCLCPP_INFO(this->get_logger(), "Mobility Stop Received");
            break;
        }

        }
    }

    void canReaderThread()
    {
        while (rclcpp::ok())
        {
            struct can_frame frame;
            if (readFrame(frame))
            {
                printFrame(frame);
                onCanReceived(frame);
            }
            else
            {
                std::this_thread::sleep_for(10ms);
            }
        }
    }

    void canWriterThread()
    {
        while (rclcpp::ok())
        {
            if (!can_send_queue_.empty() && can_socket_ != -1)
            {
                struct can_frame frame = can_send_queue_.front();
                can_send_queue_.pop();
                sendFrame(frame);
            }
            else
            {
                std::this_thread::sleep_for(10ms);
            }
        }
    }

private:
    // Socket variables
    int can_socket_ = -1;
    std::queue<struct can_frame> can_send_queue_;

    // Threading
    std::thread can_reader_thread_;
    std::thread can_writer_thread_;

    // Subscribers and Publishers
    rclcpp::Subscription<igvc_messages::msg::MotorInput>::SharedPtr mMotorInputSubscription;
    rclcpp::Subscription<igvc_messages::msg::SafetyLights>::SharedPtr mSafetyLightsSubscription;
    rclcpp::Publisher<igvc_messages::msg::MotorFeedback>::SharedPtr mMotorFeedbackPublisher;
};

int main(int argc, char *argv[])
{
    IGVC::Node::create_and_run<IGVCHardwareCanNode>(argc, argv);
}
