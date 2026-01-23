#ifndef IGVC_CPP_SHARED_LAYOUT_H
#define IGVC_CPP_SHARED_LAYOUT_H

#pragma once

constexpr int WIDTH  = 640;
constexpr int HEIGHT = 480;

constexpr int RGB_CHANNELS   = 3; // BGR8
constexpr int DEPTH_CHANNELS = 2; // Z16

constexpr std::size_t RGB_SIZE   = WIDTH * HEIGHT * RGB_CHANNELS;
constexpr std::size_t DEPTH_SIZE = WIDTH * HEIGHT * DEPTH_CHANNELS;

#pragma pack(push, 1)
struct FrameHeader
{
    int64_t sequence;
    int64_t lastWriteTicks;

    int32_t width;
    int32_t height;

    int32_t rgbStride;
    int32_t depthStride;

    int32_t rgbSizeBytes;
    int32_t depthSizeBytes;
};
#pragma pack(pop)

constexpr std::size_t SHM_SIZE =
    sizeof(FrameHeader) +
    RGB_SIZE +
    DEPTH_SIZE;

#endif //IGVC_CPP_SHARED_LAYOUT_H