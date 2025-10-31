#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"
#include "igvc/node.hpp"

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
        initCan("can0");

        can_reader_thread_ = std::thread(&IGVCHardwareCanNode::canReaderThread, this);
        can_writer_thread_ = std::thread(&IGVCHardwareCanNode::canWriterThread, this);
    }
    
    private:
    void initCan(std::string device)
    {
        struct ifreq ifr;
        struct sockaddr_can addr;
    
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error while opening CAN socket");
            return;
        }

        std::strncpy(ifr.ifr_name, device.c_str(), IFNAMSIZ - 1);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error getting interface index for %s", device.c_str());
            return;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error in socket bind for %s", device.c_str());
            return;
        }
    }

    bool readFrame(struct can_frame &frame)
    {
        ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            RCLCPP_WARN(this->get_logger(), "CAN read error");
            return false;
        }

        if (nbytes < (ssize_t)sizeof(struct can_frame)) {
            RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame");
            return false;
        }

        return true;
    }

    void sendFrame(const struct can_frame &frame)
    {
        ssize_t nbytes = write(can_socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            RCLCPP_WARN(this->get_logger(), "CAN write error");
        } else if (nbytes < (ssize_t)sizeof(struct can_frame)) {
            RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame sent");
        }
    }

    void printFrame(const struct can_frame &frame)
    {
        std::ostringstream oss;
        oss << "CAN ID: " << std::hex << frame.can_id << " DLC: " << std::dec << (int)frame.can_dlc << " Data: ";
        for (int i = 0; i < frame.can_dlc; i++) {
            oss << std::hex << (int)frame.data[i] << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    void onCanReceived(const struct can_frame &frame)
    {
    }
    
    void canReaderThread()
    {
        while (rclcpp::ok()) {
            struct can_frame frame;
            if (readFrame(frame)) {
                printFrame(frame);
                onCanReceived(frame);
            } else {
                std::this_thread::sleep_for(10ms);
            }
        }
    }

    void canWriterThread()
    {
        while (rclcpp::ok()) {
            if (!can_send_queue_.empty() && can_socket_ != -1) {
                struct can_frame frame = can_send_queue_.front();
                can_send_queue_.pop();
                sendFrame(frame);
            } else {
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
};

int main(int argc, char *argv[])
{
    IGVC::Node::create_and_run<IGVCHardwareCanNode>(argc, argv);
}
