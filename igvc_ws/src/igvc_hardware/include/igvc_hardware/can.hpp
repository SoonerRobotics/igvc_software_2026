#pragma once

namespace IGVC
{
    namespace CAN
    {
        namespace IDS
        {
            // To CAN
            constexpr int MOTOR_INPUT = 0x01;

            // From CAN
            constexpr int MOTOR_FEEDBACK = 0x02;
        }
    }
}