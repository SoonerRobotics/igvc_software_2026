#pragma once

#include <cstdint>
#include "igvc/json.hpp"
#include "igvc/defs.hpp"

namespace IGVC
{
    namespace Display
    {
        /**
         * @enum PacketType
         */
        enum class PacketType : uint8_t
        {
            UpdateSystemStatePacket = 0,
            ClientInitializationPacket = 1,
            TestPacket = 2,
        };

        /**
         * @struct PacketHeader
         * @brief Header for packets sent over WebSocket
         */
        struct PacketHeader
        {
            PacketType type;

            NLOHMANN_DEFINE_TYPE_INTRUSIVE(PacketHeader, type)
        };

        /**
         * @struct UpdateSystemStatePacket
         * @brief Packet for updating system state
         */
        struct UpdateSystemStatePacket
        {
            PacketHeader header;
            IGVC::SystemState state;

            NLOHMANN_DEFINE_TYPE_INTRUSIVE(UpdateSystemStatePacket, header, state)
        };

        /**
         * @struct ClientInitializationPacket
         * @brief Packet for client initialization
         */
        struct ClientInitializationPacket
        {
            PacketHeader header;
            IGVC::SystemState systemState;
            std::map<std::string, IGVC::DeviceState> deviceStates;

            NLOHMANN_DEFINE_TYPE_INTRUSIVE(ClientInitializationPacket, header, systemState, deviceStates)
        };

        struct TestPacket
        {
            PacketHeader header;
            std::string time;

            NLOHMANN_DEFINE_TYPE_INTRUSIVE(TestPacket, header, time)
        };
    }
}