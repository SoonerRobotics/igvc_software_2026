#pragma once

#include <cstdint>
#include "sensor_msgs/msg/joy.hpp"

namespace IGVC
{
    namespace JOY
    {
        // enum for buttons
        enum Button : std::uint8_t
        {
            A = 0,
            B = 1,
            X = 2,
            Y = 3,
            BACK = 4,
            GUIDE = 5,
            START = 6,
            LEFT_STICK = 7,
            RIGHT_STICK = 8,
            LEFT_SHOULDER = 9,
            RIGHT_SHOULDER = 10,
            DPAD_UP = 11,
            DPAD_DOWN = 12,
            DPAD_LEFT = 13,
            DPAD_RIGHT = 14,
            MISC1 = 15,
            PADDLE1 = 16,
            PADDLE2 = 17,
            PADDLE3 = 18,
            PADDLE4 = 19,
            TOUCHPAD = 20
        };

        // enum for axes
        enum Axis : std::uint8_t
        {
            LEFT_STICK_X = 0,
            LEFT_STICK_Y = 1,
            RIGHT_STICK_X = 2,
            RIGHT_STICK_Y = 3,
            LEFT_TRIGGER = 4,
            RIGHT_TRIGGER = 5
        };

        struct ControllerButtons
        {
            bool a;
            bool b;
            bool x;
            bool y;
            bool back;
            bool guide;
            bool start;
            bool left_stick;
            bool right_stick;
            bool left_shoulder;
            bool right_shoulder;
            bool dpad_up;
            bool dpad_down;
            bool dpad_left;
            bool dpad_right;
            bool misc1;
            bool paddle1;
            bool paddle2;
            bool paddle3;
            bool paddle4;
            bool touchpad;
        };

        struct ControllerAxes
        {
            float left_stick_x;
            float left_stick_y;
            float right_stick_x;
            float right_stick_y;
            float left_trigger;
            float right_trigger;
        };

        struct ControllerState
        {
            ControllerButtons buttons;
            ControllerAxes axes;
        };

        // static method to extract controller state from sensor_msgs::msg::Joy
        static ControllerState parseJoyMessage(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            // TODO: Add bounds checking for buttons and axes vectors
            ControllerState state;
            state.buttons.a = msg->buttons[Button::A];
            state.buttons.b = msg->buttons[Button::B];
            state.buttons.x = msg->buttons[Button::X];
            state.buttons.y = msg->buttons[Button::Y];
            state.buttons.back = msg->buttons[Button::BACK];
            state.buttons.guide = msg->buttons[Button::GUIDE];
            state.buttons.start = msg->buttons[Button::START];
            state.buttons.left_stick = msg->buttons[Button::LEFT_STICK];
            state.buttons.right_stick = msg->buttons[Button::RIGHT_STICK];
            state.buttons.left_shoulder = msg->buttons[Button::LEFT_SHOULDER];
            state.buttons.right_shoulder = msg->buttons[Button::RIGHT_SHOULDER];
            state.buttons.dpad_up = msg->buttons[Button::DPAD_UP];
            state.buttons.dpad_down = msg->buttons[Button::DPAD_DOWN];
            state.buttons.dpad_left = msg->buttons[Button::DPAD_LEFT];
            state.buttons.dpad_right = msg->buttons[Button::DPAD_RIGHT];
            state.buttons.misc1 = msg->buttons[Button::MISC1];
            state.buttons.paddle1 = msg->buttons[Button::PADDLE1];
            state.buttons.paddle2 = msg->buttons[Button::PADDLE2];
            state.buttons.paddle3 = msg->buttons[Button::PADDLE3];
            state.buttons.paddle4 = msg->buttons[Button::PADDLE4];
            state.buttons.touchpad = msg->buttons[Button::TOUCHPAD];

            state.axes.left_stick_x = msg->axes[Axis::LEFT_STICK_X];
            state.axes.left_stick_y = msg->axes[Axis::LEFT_STICK_Y];
            state.axes.right_stick_x = msg->axes[Axis::RIGHT_STICK_X];
            state.axes.right_stick_y = msg->axes[Axis::RIGHT_STICK_Y];
            state.axes.left_trigger = msg->axes[Axis::LEFT_TRIGGER];
            state.axes.right_trigger = msg->axes[Axis::RIGHT_TRIGGER];
            return state;
        }
    }
}