#include <chrono>
#include <map>
#include <thread>

#include "igvc/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "igvc_messages/msg/gps_feedback.hpp"
#include "igvc_messages/msg/imu_feedback.hpp"

#include "vectornav/Interface/Sensor.hpp"

using namespace std::chrono_literals;

class IGVCHardwareVectornavNode : public IGVC::Node
{
public:
    IGVCHardwareVectornavNode() : IGVC::Node("igvc_vectornav") {}

    void init() override
    {
        mGpsFeedbackPublisher = this->create_publisher<igvc_messages::msg::GPSFeedback>(IGVC::Topics::GPS, 10);
        mImuFeedbackPublisher = this->create_publisher<igvc_messages::msg::IMUFeedback>(IGVC::Topics::IMU, 10);

        setDeviceState(IGVC::DeviceState::READY);

        mSensorConnectionThread = std::thread(&IGVCHardwareVectornavNode::sensorThreadFunction, this);
        mSensorMonitorThread = std::thread(&IGVCHardwareVectornavNode::monitorThreadFunction, this);
    }

    void sensorThreadFunction()
    {
        // setup output register
        mOutputRegister.rateDivisor = 5; // 20 Hz
        mOutputRegister.asyncMode = VN::Registers::System::BinaryOutput::AsyncMode{};
        mOutputRegister.asyncMode->serial1 = 1;

        // gnss/lla
        mOutputRegister.gnss = VN::Registers::System::BinaryOutput::Gnss{};
        mOutputRegister.gnss.gnss1Fix = 1;
        mOutputRegister.gnss.gnss1NumSats = 1;
        mOutputRegister.gnss.gnss1PosLla = 1;

        // ypr
        mOutputRegister.common = VN::Registers::System::BinaryOutput::Common{};
        mOutputRegister.common.ypr = 1;

        while (rclcpp::ok())
        {
            // first, if we have a sensor check if its still connected
            if (mSensor && !mSensor->verifySensorConnectivity())
            {
                RCLCPP_WARN(get_logger(), "VectorNav sensor disconnected, attempting to reconnect");
                mSensor.reset();
                setDeviceState(IGVC::DeviceState::READY);
            }

            // if we are already connected, just sleep and continue
            if (mSensor)
            {
                std::this_thread::sleep_for(1s);
                continue;
            }

            // try to connect
            mSensor = std::make_shared<VN::Sensor>();
            VN::Error err = mSensor->autoConnect("/dev/igvc_vn");
            if (err != VN::Error::None)
            {
                std::string errString = VN::errorCodeToString(err);
                RCLCPP_WARN(get_logger(), "Failed to connect to VectorNav sensor: %s", errString.c_str());
                mSensor.reset();
                std::this_thread::sleep_for(3s);
                continue;
            }

            // setup
            mSensor->asyncOutputEnable(VN::AsyncOutputEnable::State::Disable);

            auto writeCmdOpt = mOutputRegister.toWriteCommand();
            if (!writeCmdOpt)
            {
                RCLCPP_ERROR(get_logger(), "Failed to create VectorNav output register write command");
                mSensor.reset();
                std::this_thread::sleep_for(3s);
                continue;
            }

            mSensor->asyncOutputEnable(VN::AsyncOutputEnable::State::Enable);

            // connected
            RCLCPP_INFO(get_logger(), "Connected to VectorNav sensor");
            setDeviceState(IGVC::DeviceState::OPERATING);
            mIsConnected.store(true);
        }
    }

    void monitorThreadFunction()
    {
        while (rclcpp::ok())
        {
            if (!mIsConnected.load())
            {
                std::this_thread::sleep_for(100ms);
                continue;
            }

            // read the registers
            VN::Sensor::CompositeDataQueueReturn compositeData = mSensor->getNextMeasurement();
            if (!compositeData)
            {
                // TOOD: Log?
                continue;
            }

            if (!compositeData->matchesMessage(mOutputRegister))
            {
                // we don't care about this
                continue;
            }

            // gps
            igvc_messages::msg::GPSFeedback gpsMsg;
            gpsMsg.latitude = compositeData->gnss.gnss1PosLla->lat;
            gpsMsg.longitude = compositeData->gnss.gnss1PosLla->lon;
            gpsMsg.altitude = compositeData->gnss.gnss1PosLla->alt;
            gpsMsg.num_sats = compositeData->gnss.gnss1NumSats.value_or(0);
            gpsMsg.gps_fix = compositeData->gnss.gnss1Fix.value_or(0);
            mGpsFeedbackPublisher->publish(gpsMsg);

            // imu
            igvc_messages::msg::IMUFeedback imuMsg;
            imuMsg.yaw = compositeData->attitude.ypr->yaw;
            imuMsg.pitch = compositeData->attitude.ypr->pitch;
            imuMsg.roll = compositeData->attitude.ypr->roll;
            mImuFeedbackPublisher->publish(imuMsg);

            std::this_thread::sleep_for(10ms);
        }
    }

private:
    // threads
    std::thread mSensorConnectionThread;
    std::thread mSensorMonitorThread;

    // shared data
    std::atomic<bool> mIsConnected{false};

    // vn stuff
    std::shared_ptr<VN::Sensor> mSensor;
    VN::Registers::System::BinaryOutput1 mOutputRegister;

    // publishers
    rclcpp::Publisher<igvc_messages::msg::GPSFeedback>::SharedPtr mGpsFeedbackPublisher;
    rclcpp::Publisher<igvc_messages::msg::IMUFeedback>::SharedPtr mImuFeedbackPublisher;
};

int main(int argc, char *argv[])
{
    IGVC::Node::create_and_run<IGVCHardwareVectornavNode>(argc, argv);
}
