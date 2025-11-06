#include "igvc/node.hpp"
#include "igvc/utilities.hpp"
#include "igvc/json.hpp"
#include "igvc_display/packets.hpp"
#include "igvc_display/limiter.hpp"

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <vector>
#include <cstdint>

#include "igvc_display/websocketpp/config/asio_no_tls.hpp"
#include "igvc_display/websocketpp/server.hpp"
#include <iostream>

class IGVCDataAdapter : public IGVC::Node
{
public:
    IGVCDataAdapter() : IGVC::Node("igvc_data_adapter")
    {
        mLimiter.setLimit(IGVC::LimiterKey::MotorInput, 5.0)
            .setLimit(IGVC::LimiterKey::MotorFeedback, 5.0)
            .setLimit(IGVC::LimiterKey::GPSData, 1.0);
    }

    void init() override
    {
        setDeviceState(IGVC::DeviceState::READY);

        mServerThread = std::thread([this]() { createServer(9002); });
    }

    void broadcastPacket(const nlohmann::json &packet)
    {
        std::string message = packet.dump();
        std::lock_guard<std::mutex> lock(mConnectionsMutex);
        for (auto const &hdl : mConnections)
        {
            mServer->send(hdl, message, websocketpp::frame::opcode::text);
        }
    }

    void onMessage(websocketpp::connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg)
    {
        // extract the PacketHeader from nlohmann json (packet["header"])
        nlohmann::json json_msg = nlohmann::json::parse(msg->get_payload());
        IGVC::Display::PacketHeader header = json_msg["header"].get<IGVC::Display::PacketHeader>();
        RCLCPP_INFO(this->get_logger(), "Received packet of type %d", static_cast<uint8_t>(header.type));

        switch (header.type)
        {
            case IGVC::Display::PacketType::UpdateSystemStatePacket:
            {
                IGVC::Display::UpdateSystemStatePacket packet = json_msg.get<IGVC::Display::UpdateSystemStatePacket>();
                RCLCPP_INFO(this->get_logger(), "Updating system state to %d", static_cast<uint8_t>(packet.state));
                setSystemState(packet.state);
                break;
            }
        }
    }

    void createServer(int port)
    {
        mServer = std::make_shared<websocketpp::server<websocketpp::config::asio>>();

        mServer->set_open_handler([this](websocketpp::connection_hdl hdl)
                                  {
            {
                std::lock_guard<std::mutex> lock(mConnectionsMutex);
                mConnections.insert(hdl);
            }

            IGVC::Display::ClientInitializationPacket packet {
                .header = {
                    .type = IGVC::Display::PacketType::ClientInitializationPacket
                },
                .systemState = getSystemState(),
                .deviceStates = getAllDeviceStates(),
            };
            nlohmann::json json_packet = packet;
            mServer->send(hdl, json_packet.dump(), websocketpp::frame::opcode::text);

            RCLCPP_INFO(this->get_logger(), "New WebSocket connection established."); });

        mServer->set_close_handler([this](websocketpp::connection_hdl hdl)
                                   {
            {
                std::lock_guard<std::mutex> lock(mConnectionsMutex);
                mConnections.erase(hdl);
            }

            RCLCPP_INFO(this->get_logger(), "WebSocket connection closed."); });

        mServer->set_message_handler([this](websocketpp::connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg)
                                     { onMessage(hdl, msg); });

        mServer->init_asio();
        mServer->listen(port);
        mServer->start_accept();

        setDeviceState(IGVC::DeviceState::OPERATING);

        RCLCPP_INFO(this->get_logger(), "WebSocket server started on port %d", port);
        mServer->run();
    }

private:
    std::thread mServerThread;
    std::shared_ptr<websocketpp::server<websocketpp::config::asio>> mServer;

    // connections
    std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> mConnections;
    std::mutex mConnectionsMutex;

    // limiter
    IGVC::Limiter mLimiter;
};

int main(int argc, char *argv[])
{
    IGVC::Node::create_and_run<IGVCDataAdapter>(argc, argv);
}