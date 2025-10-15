#pragma once
#include "igvc/json.hpp"

namespace IGVC
{
    enum SystemState
    {
        DISABLED = 0,
        MANUAL = 1,
        AUTONOMOUS = 2,
        SHUTDOWN = 3
    };

    enum DeviceState
    {
        OFF = 0,
        INITIALIZING = 1,
        READY = 2,
        OPERATING = 3
    };

    struct DeviceInitPayload
    {
        SystemState system_state;
        DeviceState device_state;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(DeviceInitPayload, system_state, device_state)
    };
}