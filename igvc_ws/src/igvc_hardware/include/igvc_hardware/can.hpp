#pragma once

#include <cstdint>

namespace IGVC
{
    namespace CAN
    {
        namespace IDS
        {
            // To CAN
            constexpr int MOTOR_INPUT = 10;
            constexpr int SAFETY_LIGHTS = 13;

            // From CAN
            constexpr int MOTOR_FEEDBACK = 14;
            constexpr int ESTOP = 0;
            constexpr int MOBILITY_STOP = 1;
            constexpr int MOBILITY_START = 9;
        }

        namespace Packets
        {
            /**
             * A MotorInput packet to control the robot.
             * Three signed shorts (forward velocity, sideways velocity, angular velocity)
             */
            struct MotorInput
            {
                int16_t forward_velocity;
                int16_t sideways_velocity;
                int16_t angular_velocity;
            };

            /**
             * A MotorFeedback packet
             * Three signed shorts (delta_x, delta_y, delta_theta)
             */
            struct MotorFeedback
            {
                int16_t delta_x;
                int16_t delta_y;
                int16_t delta_theta;
            };

            /**
             * A SafetyLights packet
             * Four bytes (r, g, b, mode)
             * r, g, b: color values (0-255)
             * mode: 0 = solid, 1 = blink, 2 = booting
             * blink_rate_ms: blink rate in milliseconds (only used if mode is blink)
             */
            struct SafetyLights
            {
                uint8_t r;
                uint8_t g;
                uint8_t b;
                uint8_t mode;
                uint16_t blink_rate_ms;
            };
        }
    }
}