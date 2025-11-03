#pragma once

#include <string>
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace IGVC
{
    namespace Util
    {
        /**
         * @brief Prepares a topic string by replacing {dynamic} with the provided argument.
         */
        std::string prepare(const std::string &topicTemplate, const std::string &dynamicPart)
        {
            std::string topic = topicTemplate;
            size_t pos = topic.find("{dynamic}");
            if (pos != std::string::npos)
            {
                topic.replace(pos, 9, dynamicPart);
            }
            return topic;
        }

        /**
         * @brief Converts a ROS2 CompressedImage message to an OpenCV Mat.
         */
        cv::Mat compressedImageMsgToMat(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception &e)
            {
                throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
            }
            return cv_ptr->image;
        }

        /**
         * @brief Converts a device state enum to a human-readable string.
         */
        std::string deviceStateToString(IGVC::DeviceState state)
        {
            switch (state)
            {
            case IGVC::DeviceState::OFF:
                return "OFF";
            case IGVC::DeviceState::INITIALIZING:
                return "INITIALIZING";
            case IGVC::DeviceState::READY:
                return "READY";
            case IGVC::DeviceState::OPERATING:
                return "OPERATING";
            case IGVC::DeviceState::UNKNOWN:
                return "UNKNOWN";
            case IGVC::DeviceState::ERRORED:
                return "ERRORED";
            default:
                return "INVALID_STATE";
            }
        }

        std::string systemStateToString(IGVC::SystemState state)
        {
            switch (state)
            {
            case IGVC::SystemState::DISABLED:
                return "DISABLED";
            case IGVC::SystemState::MANUAL:
                return "MANUAL";
            case IGVC::SystemState::AUTONOMOUS:
                return "AUTONOMOUS";
            case IGVC::SystemState::SHUTDOWN:
                return "SHUTDOWN";
            default:
                return "UNKNOWN";
            }
        }
    }
}