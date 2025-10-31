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
    }
}