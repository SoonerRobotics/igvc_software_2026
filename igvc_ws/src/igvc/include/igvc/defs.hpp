#pragma once
#include "igvc/json.hpp"

#include <cstdint>

namespace IGVC
{
    enum SystemState : uint8_t
    {
        DISABLED = 0,
        MANUAL = 1,
        AUTONOMOUS = 2,
        SHUTDOWN = 3
    };

    enum DeviceState : uint8_t
    {
        OFF = 0,
        INITIALIZING = 1,
        READY = 2,
        OPERATING = 3,
        UNKNOWN = 4,
        ERRORED = 5,
    };

    struct DeviceInitPayload
    {
        SystemState system_state;
        DeviceState device_state;
        std::string configuration;
        std::map<std::string, DeviceState> device_states;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(DeviceInitPayload, system_state, device_state, configuration, device_states)
    };
}