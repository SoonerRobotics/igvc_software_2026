// https://medium.com/@shahriar.rezghi.sh/using-yolo-in-c-55d55419a947

#include <chrono>
#include <tuple>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>

#include <unistd.h>

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
    IGCVVisionYoloNode()
        : IGVC::Node("igvc_vision_yolo"),
          mOrtEnv(ORT_LOGGING_LEVEL_WARNING, "yolo11"),
          mOrtSessionOptions()
    {
        mOrtSessionOptions.SetIntraOpNumThreads(1);
        mOrtSessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        // load the model
        mOrtSession = std::make_unique<Ort::Session>(mOrtEnv, "yolo11n.onnx", mOrtSessionOptions);

        auto [input_tensor_values, input_tensor_shape, image] = read_image("igvc_f13.png", 640);
        auto [output_tensor_values, output_tensor_shape] = process_image(*mOrtSession, input_tensor_values, input_tensor_shape);
        display_image(image, output_tensor_values, output_tensor_shape);
    }

    std::tuple<std::vector<float>, std::vector<long>, cv::Mat>
    read_image(const std::string &path, int size)
    {
        cv::Mat image = cv::imread(path);
        assert(!image.empty() && image.channels() == 3);

        cv::resize(image, image, {size, size});

        cv::Mat nchw = cv::dnn::blobFromImage(
            image,
            1.0 / 255.0,
            cv::Size(size, size),
            cv::Scalar(),
            true,
            false);

        std::vector<long> shape = {1, nchw.size[1], nchw.size[2], nchw.size[3]}; // NCHW
        std::vector<float> array(
            (float *)nchw.data,
            (float *)nchw.data + nchw.total());
        return {array, shape, image};
    }

    std::pair<std::vector<float>, std::vector<long>>
    process_image(Ort::Session &session, std::vector<float> &array, std::vector<long> &shape)
    {
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            array.data(),
            array.size(),
            shape.data(),
            shape.size());

        const char *input_names[] = {"images"};
        const char *output_names[] = {"output0"};

        auto output_tensors = session.Run(
            Ort::RunOptions{nullptr},
            input_names,
            &input_tensor,
            1,
            output_names,
            1);

        // get output tensor
        auto &output = output_tensors[0];
        Ort::TensorTypeAndShapeInfo output_info = output.GetTensorTypeAndShapeInfo();
        std::vector<int64_t> out_shape = output_info.GetShape();

        // convert shape to long
        std::vector<long> long_shape(out_shape.begin(), out_shape.end());
        float *ptr = output.GetTensorMutableData<float>();
        size_t total_size = 1;
        for (auto d : out_shape)
        {
            total_size *= static_cast<size_t>(d);
        }

        std::vector<float> output_data(ptr, ptr + total_size);
        return {output_data, long_shape};
    }

    void display_image(cv::Mat image,
                       const std::vector<float> &output,
                       const std::vector<long> &shape)
    {
        // shape is: [1, C, N] where C = 4 + num_classes, N = num_predictions
        if (shape.size() != 3 || shape[0] != 1)
        {
            RCLCPP_ERROR(get_logger(),
                         "Unexpected YOLO output shape. Expected [1, C, N], got [%ld, %ld, %ld]",
                         shape.size() > 0 ? shape[0] : -1,
                         shape.size() > 1 ? shape[1] : -1,
                         shape.size() > 2 ? shape[2] : -1);
            return;
        }

        // parse shape
        const int64_t batch = shape[0];
        const int64_t channels = shape[1];
        const int64_t num_preds = shape[2];
        if (channels <= 4)
        {
            return;
        }

        // COCO class names
        const int num_classes = static_cast<int>(channels - 4);
        static const char *class_names[] = {
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

        const int coco_num_classes = static_cast<int>(sizeof(class_names) / sizeof(class_names[0]));

        struct Detection
        {
            float x1, y1, x2, y2;
            float score;
            int class_id;
        };

        const float conf_threshold = 0.25f;
        const float iou_threshold = 0.45f;

        std::vector<Detection> detections;
        detections.reserve(num_preds);

        const float *data = output.data();
        for (int64_t n = 0; n < num_preds; ++n)
        {
            float x = data[0 * num_preds + n];
            float y = data[1 * num_preds + n];
            float w = data[2 * num_preds + n];
            float h = data[3 * num_preds + n];

            int best_class = -1;
            float best_score = 0.0f;

            for (int c = 0; c < num_classes; ++c)
            {
                float score = data[(4 + c) * num_preds + n];
                if (score > best_score)
                {
                    best_score = score;
                    best_class = c;
                }
            }

            if (best_score < conf_threshold || best_class < 0)
                continue;

            float x1 = x - w / 2.0f;
            float y1 = y - h / 2.0f;
            float x2 = x + w / 2.0f;
            float y2 = y + h / 2.0f;

            x1 = std::max(0.0f, std::min(x1, static_cast<float>(image.cols - 1)));
            x2 = std::max(0.0f, std::min(x2, static_cast<float>(image.cols - 1)));
            y1 = std::max(0.0f, std::min(y1, static_cast<float>(image.rows - 1)));
            y2 = std::max(0.0f, std::min(y2, static_cast<float>(image.rows - 1)));

            if (x2 <= x1 || y2 <= y1)
                continue;

            detections.push_back({x1, y1, x2, y2, best_score, best_class});
        }

        auto iou = [](const Detection &a, const Detection &b) -> float
        {
            float inter_x1 = std::max(a.x1, b.x1);
            float inter_y1 = std::max(a.y1, b.y1);
            float inter_x2 = std::min(a.x2, b.x2);
            float inter_y2 = std::min(a.y2, b.y2);

            float inter_w = std::max(0.0f, inter_x2 - inter_x1);
            float inter_h = std::max(0.0f, inter_y2 - inter_y1);
            float inter_area = inter_w * inter_h;

            float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
            float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);

            float denom = area_a + area_b - inter_area + 1e-6f;
            return inter_area / denom;
        };

        std::sort(detections.begin(), detections.end(),
                  [](const Detection &a, const Detection &b)
                  { return a.score > b.score; });

        std::vector<Detection> final_dets;
        std::vector<bool> suppressed(detections.size(), false);

        for (size_t i = 0; i < detections.size(); ++i)
        {
            if (suppressed[i])
                continue;

            const Detection &det_i = detections[i];
            final_dets.push_back(det_i);

            for (size_t j = i + 1; j < detections.size(); ++j)
            {
                if (suppressed[j])
                    continue;

                const Detection &det_j = detections[j];
                if (det_i.class_id != det_j.class_id)
                    continue;

                if (iou(det_i, det_j) > iou_threshold)
                {
                    suppressed[j] = true;
                }
            }
        }

        for (const auto &det : final_dets)
        {
            int x = static_cast<int>(det.x1);
            int y = static_cast<int>(det.y1);
            int w = static_cast<int>(det.x2 - det.x1);
            int h = static_cast<int>(det.y2 - det.y1);
            int c = det.class_id;

            std::string cls_name;
            if (c >= 0 && c < coco_num_classes)
                cls_name = class_names[c];
            else
                cls_name = "cls_" + std::to_string(c);

            std::string label = cls_name + ":" + std::to_string(static_cast<int>(det.score * 100)) + "%";

            cv::Scalar color = cv::Scalar(255, 255, 255);
            cv::rectangle(image, cv::Rect(x, y, w, h), color, 2);
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_DUPLEX, 0.7, 1, &baseline);
            cv::rectangle(image,
                          cv::Rect(x, y - text_size.height - 4, text_size.width + 4, text_size.height + 4),
                          color, cv::FILLED);
            cv::putText(image, label,
                        cv::Point(x + 2, y - 4),
                        cv::FONT_HERSHEY_DUPLEX, 0.7,
                        cv::Scalar(0, 0, 0), 1);
        }

        cv::imshow("YOLOv11 Output", image);
        cv::waitKey(0);
    }

    void init() override
    {
        RCLCPP_INFO(get_logger(), "Initializing YOLOv11 Vision Node");
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
