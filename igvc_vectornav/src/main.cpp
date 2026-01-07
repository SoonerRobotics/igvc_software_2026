#include "spdlog/spdlog.h"
#include "vectornav/Interface/Sensor.hpp"

#include "igvc/flatbuffers/VectornavReport_cpp_generated.h"

std::optional<VN::Registers::System::BinaryOutput1> setupSensor(std::shared_ptr<VN::Sensor> &sensor)
{
    VN::Registers::System::BinaryOutput1 mOutputRegister;
    mOutputRegister.rateDivisor = 5;                                              // 20 Hz output rate at 100 Hz base rate
    mOutputRegister.asyncMode = VN::Registers::System::BinaryOutput::AsyncMode{}; // Clear all async modes
    mOutputRegister.asyncMode->serial1 = 1;                                       // Enable Serial1 async output

    // gnss/lla
    mOutputRegister.gnss = VN::Registers::System::BinaryOutput::Gnss{};
    mOutputRegister.gnss.gnss1PosLla = 1;
    mOutputRegister.gnss.gnss1VelNed = 1;
    mOutputRegister.gnss.gnss1Fix = 1;
    mOutputRegister.gnss.gnss1NumSats = 1;

    // ypr
    mOutputRegister.common = VN::Registers::System::BinaryOutput::Common{};
    mOutputRegister.common.ypr = 1;

    // Disable output
    sensor->asyncOutputEnable(VN::AsyncOutputEnable::State::Disable);

    // Write our stuff
    auto cmd = mOutputRegister.toWriteCommand();
    if (!cmd.has_value())
    {
        spdlog::error("Failed to create write command for BinaryOutput1 register.");
        return std::nullopt;
    }

    VN::Error writeError = sensor->sendCommand(&(cmd.value()), VN::Sensor::SendCommandBlockMode::Block);
    if (writeError != VN::Error::None)
    {
        spdlog::error("Failed to write BinaryOutput1 register. Error code: {}", errorCodeToString(writeError));
        return std::nullopt;
    }

    // Re-enable output
    sensor->asyncOutputEnable(VN::AsyncOutputEnable::State::Enable);

    return mOutputRegister;
}

int main()
{
    spdlog::info("Starting IGVC VectorNav Interface");

    std::shared_ptr<VN::Sensor> sensor = std::make_shared<VN::Sensor>();
    VN::Error connectError = sensor->autoConnect("/dev/ttyUSB0");
    if (connectError != VN::Error::None)
    {
        spdlog::error("Failed to connect to VectorNav sensor. Error code: {}", errorCodeToString(connectError));
        return -1;
    }

    // Print the current baudrate
    auto connectedBaudRate = sensor->connectedBaudRate().value();
    spdlog::info("Connected to VectorNav sensor at baud rate: {}", static_cast<uint16_t>(connectedBaudRate));

    // Setup sensor configuration
    auto reg = setupSensor(sensor);
    if (!reg.has_value())
    {
        spdlog::error("Failed to setup VectorNav sensor.");
        return -1;
    }

    // Main loop to read data
    while (true)
    {
        VN::Sensor::CompositeDataQueueReturn compositeData = sensor->getNextMeasurement();
        if (!compositeData)
        {
            continue;
        }

        if (!compositeData->matchesMessage(reg.value()))
        {
            // Not the message we're looking for
            continue;
        }

        // GPS stuff
        auto latitude = compositeData->gnss.gnss1PosLla->lat;
        auto longitude = compositeData->gnss.gnss1PosLla->lon;
        auto altitude = compositeData->gnss.gnss1PosLla->alt;
        auto numSats = compositeData->gnss.gnss1NumSats.value_or(0);
        auto gpsFix = compositeData->gnss.gnss1Fix.value_or(0);

        // YPR stuff
        auto yaw = compositeData->attitude.ypr->yaw;
        auto pitch = compositeData->attitude.ypr->pitch;
        auto roll = compositeData->attitude.ypr->roll;

        // Create the Flatbuffers message
        flatbuffers::FlatBufferBuilder builder(1024);

        igvc::Location location (latitude, longitude);
        igvc::Orientation orientation (yaw, pitch, roll);

        auto report = igvc::CreateVectornavReport(
            builder,
            0L,
            0,
            &location,
            &orientation,
            static_cast<uint16_t>(numSats),
            static_cast<int8_t>(gpsFix)
        );

        builder.Finish(report);

        // Do stuff with it
        uint8_t* buf = builder.GetBufferPointer();
        size_t size = builder.GetSize();

        spdlog::info("Vectornav Report - Lat: {}, Lon: {}, Alt: {}, Yaw: {}, Pitch: {}, Roll: {}, Sats: {}, Fix: {}",
            latitude, longitude, altitude, yaw, pitch, roll, numSats, gpsFix);
    }

    sensor->disconnect();
    return 0;
}