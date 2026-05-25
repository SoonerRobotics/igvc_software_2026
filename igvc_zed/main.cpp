#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <unistd.h>
#include <optional>

#include "spdlog/spdlog.h"
#include <sl/Camera.hpp>

static constexpr int FRAME_WIDTH = 1280;
static constexpr int FRAME_HEIGHT = 720;
static constexpr int FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT * 4; // BGRA

static const char *SHM_FRAME_NAME = "/zed_frame";
static const char *SEM_FRAME_NAME = "/zed_frame_sem";
static const char *SHM_DEPTH_NAME = "/zed_depth";
static const char *SEM_DEPTH_NAME = "/zed_depth_sem";

#pragma pack(push, 1)
struct ZedFrameHeader
{
    uint32_t sequenceNum;
    int64_t timestampUs;
    uint32_t width;
    uint32_t height;
    uint8_t valid;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ZedDepthRequest
{
    uint32_t requestId;
    int32_t pixelX;
    int32_t pixelY;
};

struct ZedDepthResponse
{
    uint32_t requestId;
    float x;
    float y;
    float z;
    float distance;
    uint8_t valid;
};

struct ZedDepthChannel
{
    ZedDepthRequest request;
    ZedDepthResponse response;
};
#pragma pack(pop)

static std::atomic<bool> g_running{true};

void signalHandler(int sig)
{
    spdlog::warn("RECEIVED_SIGNAL_{}", sig);
    g_running = false;
}

struct FrameShmContext
{
    uint8_t *ptr = nullptr;
    sem_t *sem = SEM_FAILED;
    size_t totalSize = 0;
};

struct DepthShmContext
{
    ZedDepthChannel *ptr = nullptr;
    sem_t *sem = SEM_FAILED;
};

FrameShmContext openFrameShm()
{
    FrameShmContext ctx;
    ctx.totalSize = sizeof(ZedFrameHeader) + FRAME_BYTES;

    int fd = shm_open(SHM_FRAME_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0)
    {
        spdlog::error("SHM_FRAME_OPEN_FAILED_{}", strerror(errno));
        return ctx;
    }

    if (ftruncate(fd, static_cast<off_t>(ctx.totalSize)) < 0)
    {
        spdlog::error("SHM_FRAME_TRUNCATE_FAILED_{}", strerror(errno));
        close(fd);
        return ctx;
    }

    void *raw = mmap(nullptr, ctx.totalSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (raw == MAP_FAILED)
    {
        spdlog::error("SHM_FRAME_MMAP_FAILED_{}", strerror(errno));
        return ctx;
    }

    ctx.ptr = static_cast<uint8_t *>(raw);
    memset(ctx.ptr, 0, ctx.totalSize);

    ctx.sem = sem_open(SEM_FRAME_NAME, O_CREAT, 0666, 1);
    if (ctx.sem == SEM_FAILED)
    {
        spdlog::error("SEM_FRAME_OPEN_FAILED_{}", strerror(errno));
        munmap(raw, ctx.totalSize);
        ctx.ptr = nullptr;
    }

    return ctx;
}

DepthShmContext openDepthShm()
{
    DepthShmContext ctx;

    int fd = shm_open(SHM_DEPTH_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0)
    {
        spdlog::error("SHM_DEPTH_OPEN_FAILED_{}", strerror(errno));
        return ctx;
    }

    if (ftruncate(fd, static_cast<off_t>(sizeof(ZedDepthChannel))) < 0)
    {
        spdlog::error("SHM_DEPTH_TRUNCATE_FAILED_{}", strerror(errno));
        close(fd);
        return ctx;
    }

    void *raw = mmap(nullptr, sizeof(ZedDepthChannel), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (raw == MAP_FAILED)
    {
        spdlog::error("SHM_DEPTH_MMAP_FAILED_{}", strerror(errno));
        return ctx;
    }

    ctx.ptr = static_cast<ZedDepthChannel *>(raw);
    memset(ctx.ptr, 0, sizeof(ZedDepthChannel));

    ctx.sem = sem_open(SEM_DEPTH_NAME, O_CREAT, 0666, 1);
    if (ctx.sem == SEM_FAILED)
    {
        spdlog::error("SEM_DEPTH_OPEN_FAILED_{}", strerror(errno));
        munmap(raw, sizeof(ZedDepthChannel));
        ctx.ptr = nullptr;
    }

    return ctx;
}

void closeFrameShm(FrameShmContext &ctx)
{
    if (ctx.ptr)
    {
        munmap(ctx.ptr, ctx.totalSize);
        ctx.ptr = nullptr;
    }
    if (ctx.sem != SEM_FAILED)
    {
        sem_close(ctx.sem);
        ctx.sem = SEM_FAILED;
    }
}

void closeDepthShm(DepthShmContext &ctx)
{
    if (ctx.ptr)
    {
        munmap(ctx.ptr, sizeof(ZedDepthChannel));
        ctx.ptr = nullptr;
    }
    if (ctx.sem != SEM_FAILED)
    {
        sem_close(ctx.sem);
        ctx.sem = SEM_FAILED;
    }
}

void cleanupSharedMemory()
{
    shm_unlink(SHM_FRAME_NAME);
    sem_unlink(SEM_FRAME_NAME);
    shm_unlink(SHM_DEPTH_NAME);
    sem_unlink(SEM_DEPTH_NAME);
}

bool semTimedWait(sem_t *sem, int timeoutMs = 100)
{
    struct timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += static_cast<long>(timeoutMs) * 1'000'000L;
    if (ts.tv_nsec >= 1'000'000'000L)
    {
        ts.tv_sec++;
        ts.tv_nsec -= 1'000'000'000L;
    }
    return sem_timedwait(sem, &ts) == 0;
}

void writeFrame(FrameShmContext &ctx, const sl::Mat &image, uint32_t seqNum)
{
    if (!ctx.ptr || ctx.sem == SEM_FAILED)
        return;

    if (!semTimedWait(ctx.sem))
    {
        spdlog::warn("SEM_FRAME_TIMEDWAIT_FAILED");
        return;
    }

    auto *header = reinterpret_cast<ZedFrameHeader *>(ctx.ptr);
    uint8_t *pixels = ctx.ptr + sizeof(ZedFrameHeader);

    auto now = std::chrono::system_clock::now().time_since_epoch();
    header->timestampUs = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
    header->sequenceNum = seqNum;
    header->width = FRAME_WIDTH;
    header->height = FRAME_HEIGHT;
    header->valid = 1;

    memcpy(pixels, image.getPtr<sl::uchar1>(), FRAME_BYTES);

    sem_post(ctx.sem);
}

void handleDepthRequest(DepthShmContext &ctx, sl::Camera &zed)
{
    if (!ctx.ptr || ctx.sem == SEM_FAILED)
    {
        return;
    }

    if (sem_trywait(ctx.sem) != 0)
    {
        return;
    }

    ZedDepthChannel local;
    memcpy(&local, ctx.ptr, sizeof(ZedDepthChannel));

    if (local.request.requestId == 0 || local.request.requestId == local.response.requestId)
    {
        sem_post(ctx.sem);
        return;
    }

    sem_post(ctx.sem);

    sl::float4 point3D;
    sl::Mat depthMap;
    sl::ERROR_CODE err = zed.retrieveMeasure(depthMap, sl::MEASURE::XYZ, sl::MEM::CPU, sl::Resolution(FRAME_WIDTH, FRAME_HEIGHT));

    ZedDepthResponse resp{};
    resp.requestId = local.request.requestId;

    if (err == sl::ERROR_CODE::SUCCESS)
    {
        depthMap.getValue(local.request.pixelX, local.request.pixelY, &point3D);

        bool validPoint = std::isfinite(point3D.x) &&
                          std::isfinite(point3D.y) &&
                          std::isfinite(point3D.z);

        if (validPoint)
        {
            resp.x = point3D.x;
            resp.y = point3D.y;
            resp.z = point3D.z;
            resp.distance = std::sqrt(point3D.x * point3D.x +
                                      point3D.y * point3D.y +
                                      point3D.z * point3D.z);
            resp.valid = 1;
        }
        else
        {
            spdlog::warn("DEPTH_POINT_INVALID_({},{})", local.request.pixelX, local.request.pixelY);
            resp.valid = 0;
        }
    }
    else
    {
        spdlog::warn("DEPTH_RETRIEVE_FAILED_{}", sl::toString(err).get());
        resp.valid = 0;
    }

    if (semTimedWait(ctx.sem))
    {
        ctx.ptr->response = resp;
        sem_post(ctx.sem);
    }
}

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    spdlog::set_pattern("{\"timestamp\": \"%Y-%m-%dT%H:%M:%S.%eZ\", \"level\": \"%l\", \"name\": \"%n\", \"message\": \"%v\"}");
    spdlog::info("ZED_STARTING");

    FrameShmContext frameSHM = openFrameShm();
    if (!frameSHM.ptr)
    {
        spdlog::error("FRAME_SHM_INIT_FAILED");
        return 1;
    }

    DepthShmContext depthSHM = openDepthShm();
    if (!depthSHM.ptr)
    {
        spdlog::error("DEPTH_SHM_INIT_FAILED");
        closeFrameShm(frameSHM);
        return 1;
    }

    uint32_t seqNum = 0;

    constexpr auto frameRate = 15;
    constexpr auto framePeriod = std::chrono::milliseconds(1000 / frameRate);
    while (g_running)
    {
        sl::Camera zed;

        sl::InitParameters initParams;
        initParams.camera_resolution = sl::RESOLUTION::HD720; // 1280×720
        initParams.camera_fps = frameRate;
        initParams.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
        initParams.coordinate_units = sl::UNIT::METER;
        initParams.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;

        sl::ERROR_CODE openErr = zed.open(initParams);
        if (openErr != sl::ERROR_CODE::SUCCESS)
        {
            spdlog::error("ZED_CONNECTION_FAILED_{}", sl::toString(openErr).get());
            std::this_thread::sleep_for(std::chrono::seconds(3));
            continue;
        }

        auto camInfo = zed.getCameraInformation();
        spdlog::info("ZED_CONNECTED_SN_{}", camInfo.serial_number);

        sl::RuntimeParameters rtParams;
        rtParams.enable_depth = true;

        sl::Mat imageLeft;

        auto nextFrameTime = std::chrono::steady_clock::now();

        while (g_running)
        {
            auto now = std::chrono::steady_clock::now();

            handleDepthRequest(depthSHM, zed);
            if (now < nextFrameTime)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            nextFrameTime = now + framePeriod;

            sl::ERROR_CODE grabErr = zed.grab(rtParams);
            if (grabErr == sl::ERROR_CODE::SUCCESS)
            {
                zed.retrieveImage(imageLeft, sl::VIEW::LEFT, sl::MEM::CPU,
                                  sl::Resolution(FRAME_WIDTH, FRAME_HEIGHT));

                writeFrame(frameSHM, imageLeft, ++seqNum);

                spdlog::debug("FRAME_WRITTEN_SEQ_{}", seqNum);
            }
            else if (grabErr == sl::ERROR_CODE::CAMERA_NOT_DETECTED)
            {
                spdlog::warn("ZED_DISCONNECTED");
                break;
            }
            else
            {
                spdlog::warn("ZED_GRAB_FAILED_{}", sl::toString(grabErr).get());
            }
        }

        spdlog::warn("ZED_DISCONNECTED");
        zed.close();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    spdlog::info("ZED_SHUTDOWN");
    closeFrameShm(frameSHM);
    closeDepthShm(depthSHM);
    cleanupSharedMemory();
    return 0;
}