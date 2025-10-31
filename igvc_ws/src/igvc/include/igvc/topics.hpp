#pragma once

namespace IGVC
{
    namespace Topics
    {
        constexpr const char *DEVICE_INIT = "/igvc/device_init";
        constexpr const char *GPS = "/igvc/gps";
        constexpr const char *CAMERA = "/igvc/camera/{dynamic}/raw";
        constexpr const char *PROCESSED_IMAGE = "/igvc/camera/{dynamic}/processed";
        constexpr const char *CONFIGURATION = "/igvc/configuration";
    }

    namespace Services
    {
        constexpr const char *UPDATE_CONFIGURATION = "/igvc/update_configuration";
    }
}