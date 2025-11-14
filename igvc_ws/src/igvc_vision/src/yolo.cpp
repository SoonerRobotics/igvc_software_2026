// https://medium.com/@shahriar.rezghi.sh/using-yolo-in-c-55d55419a947

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "igvc/node.hpp"

#include "onnxruntime_cxx_api.h"

using namespace std::chrono_literals;

class IGCVVisionYoloNode : public IGVC::Node
{
public:
    IGCVVisionYoloNode() : IGVC::Node("igvc_vision_yolo")
    {
        // print current working directory
        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL)
        {
            RCLCPP_INFO(get_logger(), "Current working directory: %s", cwd);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "getcwd() error");
        }

        mOrtSession = std::make_unique<Ort::Session>(mOrtEnv, "yolo11n.onnx", mOrtSessionOptions);

        // read in image1.png
        auto [input_tensor_values, input_tensor_shape, image] = read_image("bus.jpg", 640);

        // process the image through the model
        auto [output_tensor_values, output_tensor_shape] = process_image(*mOrtSession, input_tensor_values, input_tensor_shape);

        // display the image with bounding boxes
        display_image(image, output_tensor_values, output_tensor_shape);
    }

    std::tuple<std::vector<float>, std::vector<long>, cv::Mat> read_image(const std::string &path, int size)
    {
        auto image = cv::imread(path);
        assert(!image.empty() && image.channels() == 3);
        cv::resize(image, image, {size, size});
        std::vector<long> shape = {1, image.channels(), image.rows, image.cols};
        cv::Mat nchw = cv::dnn::blobFromImage(image, 1.0, {}, {}, true) / 255.f;
        std::vector<float> array(nchw.ptr<float>(), nchw.ptr<float>() + nchw.total());
        return {array, shape, image};
    }

    std::pair<std::vector<float>, std::vector<long>> process_image(Ort::Session &session, std::vector<float> &array, std::vector<long> &shape)
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        auto input = Ort::Value::CreateTensor<float>(
            memory_info, (float *)array.data(), array.size(), shape.data(), shape.size());

        const char *input_names[] = {"images"};
        const char *output_names[] = {"output0"};
        auto output = session.Run({}, input_names, &input, 1, output_names, 1);
        shape = output[0].GetTensorTypeAndShapeInfo().GetShape();
        auto ptr = output[0].GetTensorMutableData<float>();
        return {std::vector<float>(ptr, ptr + shape[0] * shape[1]), shape};
    }

    void display_image(cv::Mat image, const std::vector<float> &output, const std::vector<long> &shape)
    {
        const char *class_names[] = {
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
            "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
            "bird", "cat", "dog", "horse", "sheep", "cow", "elephant",
            "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie",
            "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
            "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
            "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
            "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
            "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv",
            "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
            "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
            "teddy bear", "hair drier", "toothbrush"};

        for (size_t i = 0; i < shape[0]; ++i)
        {
            auto ptr = output.data() + i * shape[1];
            int x = ptr[1], y = ptr[2], w = ptr[3] - x, h = ptr[4] - y, c = ptr[5];
            auto color = CV_RGB(255, 255, 255);
            auto name = std::string(class_names[c]) + ":" + std::to_string(int(ptr[6] * 100)) + "%";
            cv::rectangle(image, {x, y, w, h}, color);
            cv::putText(image, name, {x, y}, cv::FONT_HERSHEY_DUPLEX, 1, color);
        }

        cv::imshow("YOLOv7 Output", image);
        cv::waitKey(0);
    }

    void init() override
    {
        RCLCPP_INFO(get_logger(), "Initializing YOLO Vision Node");

        setDeviceState(IGVC::DeviceState::OPERATING);
    }

private:
    Ort::Env mOrtEnv;
    Ort::SessionOptions mOrtSessionOptions;
    std::unique_ptr<Ort::Session> mOrtSession;
};

int main(int argc, char *argv[])
{
    IGVC::Node::create_and_run<IGCVVisionYoloNode>(argc, argv);
}
