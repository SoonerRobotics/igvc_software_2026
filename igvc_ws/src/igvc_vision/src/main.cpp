#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "igvc_vision/pipeline.hpp"
#include "igvc/node.hpp"

using namespace std::chrono_literals;

class IGVCVisionTransformerNode : public IGVC::Node
{
public:
    IGVCVisionTransformerNode(std::string cameraId) : IGVC::Node("igvc_vision_transformer_" + cameraId), mCameraId(cameraId) {}

    void init() override
    {
        mPipeline = IGVC::ImagePipeline()
            // blurring
            .addProcessor(std::make_shared<IGVC::GaussianBlurProcessor>(
                mConfig.visionConfig.getGaussianBlurKernelSize(),
                mConfig.visionConfig.getGaussianBlurSigma()
            ))

            // lane hsv thresholding
            .addProcessor(std::make_shared<IGVC::HSVThresholdProcessor>(
                mConfig.visionConfig.getLaneHsvLower(),
                mConfig.visionConfig.getLaneHsvUpper()
            ));


        mImageSubscription = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            IGVC::Util::prepare(IGVC::Topics::CAMERA, mCameraId),
            10,
            std::bind(&IGVCVisionTransformerNode::onImageReceived, this, std::placeholders::_1)
        );

        mProcessedImagePublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            IGVC::Util::prepare(IGVC::Topics::PROCESSED_IMAGE, mCameraId),
            10
        );

        setDeviceState(IGVC::DeviceState::OPERATING);
    }

private:
    void onImageReceived(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        // process using the pipeline
        cv::Mat image = IGVC::Util::compressedImageMsgToMat(msg);
        cv::Mat processedImage;
        mPipeline.processImage(image, processedImage);

        // resize processedImage to resolution
        uint32_t res = mConfig.globalConfig.getMapResolution();
        cv::resize(processedImage, processedImage, cv::Size(res, res));
        processedImage /= 2;

        // flatten to a 1d array
        std::vector<int8_t> flatData;
        flatData.assign(processedImage.datastart, processedImage.dataend);

        // create and publish
        nav_msgs::msg::OccupancyGrid occupancyGridMsg;
        occupancyGridMsg.header = msg->header;
        occupancyGridMsg.data = flatData;
        mProcessedImagePublisher->publish(occupancyGridMsg);
    }

private:
    std::string mCameraId;
    IGVC::ImagePipeline mPipeline;

    // subscriptions and publishers
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr mImageSubscription;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mProcessedImagePublisher;
};

int main(int argc, char *argv[])
{
    IGVC::Node::create_and_run<IGVCVisionTransformerNode>(argc, argv, "front");
}
