#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
// #include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node
{
public:
    MinimalImagePublisher() : Node("opencv_image_publisher")
    {
        test_opencv();
    }

private:
    void test_opencv()
    {
        // create and show a test image
        cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
        cv::putText(image, "OpenCV Test Image", cv::Point(50, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::imshow("Test Image", image);
        cv::waitKey(3000); // Display the window for 3 seconds
        cv::destroyAllWindows();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    std::shared_ptr<MinimalImagePublisher> node = std::make_shared<MinimalImagePublisher>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
