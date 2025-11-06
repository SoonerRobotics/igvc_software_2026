#pragma once

namespace IGVC
{
    namespace Topics
    {
        // real robot stuff
        constexpr const char *GPS = "/igvc/gps";
        constexpr const char *CAMERA = "/igvc/camera/{dynamic}/raw";
        constexpr const char *PROCESSED_IMAGE = "/igvc/camera/{dynamic}/processed";
        constexpr const char *MOTOR_INPUT = "/igvc/motor_input";
        constexpr const char *MOTOR_FEEDBACK = "/igvc/motor_feedback";

        // framework things
        constexpr const char *DEVICE_INIT = "/igvc/device_init";
        constexpr const char *CONFIGURATION = "/igvc/configuration";
        constexpr const char *SYSTEM_STATE = "/igvc/system_state";
        constexpr const char *DEVICE_STATE = "/igvc/device_state";
    }

    namespace Services
    {
        constexpr const char *UPDATE_CONFIGURATION = "/igvc/update_configuration";
        constexpr const char *UPDATE_DEVICE_STATE = "/igvc/update_device_state";
        constexpr const char *UPDATE_SYSTEM_STATE = "/igvc/update_system_state";
    }
}