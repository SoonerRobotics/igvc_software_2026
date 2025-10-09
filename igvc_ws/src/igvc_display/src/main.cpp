#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "raylib.h"
// #include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

class RaylibExample : public rclcpp::Node
{
public:
    RaylibExample() : Node("raylib_example")
    {
        test_raylib();
    }

private:
    void test_raylib()
    {
        InitWindow(800, 600, "Raylib with ROS2");
        SetTargetFPS(60);

        while (!WindowShouldClose() && rclcpp::ok())
        {
            BeginDrawing();

            ClearBackground(RAYWHITE);
            DrawText("Hello, Raylib with ROS2!", 190, 200, 20, LIGHTGRAY);
            
            EndDrawing();
        }

        CloseWindow();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<RaylibExample> node = std::make_shared<RaylibExample>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}