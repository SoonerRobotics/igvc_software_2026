#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <cstring>

#include "include/shared_layout.h"
#include "include/shared_memory.h"
#include "include/time_utils.h"

static void LogInfo(const std::string& category, const std::string& msg)
{
    std::cerr << "INFO|" << category << "|" << msg << "\n";
}

static void LogWarn(const std::string& category, const std::string& msg)
{
    std::cerr << "WARN|" << category << "|" << msg << "\n";
}

static void LogErr(const std::string& category, const std::string& msg)
{
    std::cerr << "ERR|" << category << "|" << msg << "\n";
}

int main()
{
    try
    {
        LogInfo("STARTUP", "Initializing RealSense pipeline");

        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);

        rs2::pipeline pipe;
        pipe.start(cfg);

        rs2::align align_to_color(RS2_STREAM_COLOR);

        // Non-const because we lock/unlock
        const SharedMemory shm("realsense_frames", SHM_SIZE);

        auto* header = static_cast<FrameHeader*>(shm.Data());
        uint8_t* rgbPtr = static_cast<uint8_t*>(shm.Data()) + sizeof(FrameHeader);
        uint8_t* depthPtr = rgbPtr + RGB_SIZE;

        int64_t sequence = 0;

        LogInfo("READY", "Streaming started");

        while (true)
        {
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            rs2::video_frame color = frames.get_color_frame();
            rs2::depth_frame depth = frames.get_depth_frame();

            if (!color || !depth)
            {
                LogWarn("STREAM", "Missing color or depth frame");
                continue;
            }

            if (color.get_width() != WIDTH || color.get_height() != HEIGHT)
            {
                LogWarn("CONFIG", "Color frame size mismatch");
                continue;
            }
            if (depth.get_width() != WIDTH || depth.get_height() != HEIGHT)
            {
                LogWarn("CONFIG", "Depth frame size mismatch");
                continue;
            }

            shm.Lock();

            header->sequence = ++sequence;
            header->lastWriteTicks = GetUnixTicks();
            header->width = WIDTH;
            header->height = HEIGHT;
            header->rgbStride = WIDTH * RGB_CHANNELS;
            header->depthStride = WIDTH * DEPTH_CHANNELS;
            header->rgbSizeBytes = RGB_SIZE;
            header->depthSizeBytes = DEPTH_SIZE;

            std::memcpy(rgbPtr, color.get_data(), RGB_SIZE);
            std::memcpy(depthPtr, depth.get_data(), DEPTH_SIZE);

            shm.Unlock();
        }
    }
    catch (const rs2::error& e)
    {
        LogErr("DEVICE", std::string("RealSense error: ") + e.what());
        return 1;
    }
    catch (const std::exception& e)
    {
        LogErr("FATAL", e.what());
        return 1;
    }
}