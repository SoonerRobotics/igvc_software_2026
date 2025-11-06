#include <chrono>
#include <map>

#include "igvc/node.hpp"
#include "igvc_hardware/joy.hpp"
#include "igvc_messages/msg/motor_input.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class IGVCHardwareControllerNode : public IGVC::Node
{
public:
    IGVCHardwareControllerNode() : IGVC::Node("igvc_controller") {}

    void init() override
    {
        mJoySubscription = create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&IGVCHardwareControllerNode::onJoyMessage, this, std::placeholders::_1)
        );

        setDeviceState(IGVC::DeviceState::READY);
    }

private:
    /**
     * @brief Normalize a value from one range to another
     * @param value The value to normalize
     * @param inMin The minimum of the input range
     * @param inMax The maximum of the input range
     * @param outMin The minimum of the output range
     * @param outMax The maximum of the output range
     * @return The normalized value
     */
    double normalize(double value, double inMin, double inMax, double outMin, double outMax)
    {
        return (value - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
    }

    void onJoyMessage(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        IGVC::JOY::ControllerState state = IGVC::JOY::parseJoyMessage(msg);
        if (!mLastState.buttons.guide && state.buttons.guide)
        {
            // "guide" button (e.g. xbox home button) was pressed
            setSystemState(IGVC::SystemState::AUTONOMOUS);
            return;
        }

        if (!mLastState.buttons.start && state.buttons.start)
        {
            // "start" button was pressed
            setSystemState(IGVC::SystemState::MANUAL);
            return;
        }

        if (!mLastState.buttons.back && state.buttons.back)
        {
            setSystemState(IGVC::SystemState::DISABLED);
            return;
        }

        if (!mLastState.buttons.b && state.buttons.b)
        {
            setSystemState(IGVC::SystemState::SHUTDOWN);
            return;
        }

        if (state.axes.left_stick_x != 0 || state.axes.left_stick_y != 0 ||
            state.axes.right_stick_x != 0 || state.axes.right_stick_y != 0)
        {
            // left stick is for strafing (e.g. forward/sideways velocity), right stick is for rotation
            double strafeX = normalize(
                state.axes.left_stick_x, 
                -1.0, 
                1.0, 
                mConfig.manualControlConfig.getSidewaysSpeed().toMetersPerSecond() * -1.0, 
                mConfig.manualControlConfig.getSidewaysSpeed().toMetersPerSecond()
            );
            double strafeY = normalize(
                state.axes.left_stick_y, 
                -1.0, 
                1.0, 
                mConfig.manualControlConfig.getForwardSpeed().toMetersPerSecond() * -1.0, 
                mConfig.manualControlConfig.getForwardSpeed().toMetersPerSecond()
            );
            double rotation = normalize(
                state.axes.right_stick_x, 
                -1.0, 
                1.0, 
                mConfig.manualControlConfig.getTurnSpeed().toRadiansPerSecond() * -1.0, 
                mConfig.manualControlConfig.getTurnSpeed().toRadiansPerSecond()
            );
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr mJoySubscription;


    IGVC::JOY::ControllerState mLastState;
    std::map<std::string, rclcpp::Time> mButtonPressedAt;
};

int main(int argc, char *argv[])
{
    IGVC::Node::create_and_run<IGVCHardwareControllerNode>(argc, argv);
}
