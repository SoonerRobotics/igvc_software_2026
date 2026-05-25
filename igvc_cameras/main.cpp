#include <iostream>
#include <atomic>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <unistd.h>

#include "spdlog/spdlog.h"
#include <opencv2/opencv.hpp>

static constexpr int FRAME_WIDTH = 640;
static constexpr int FRAME_HEIGHT = 480;
static constexpr int FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT * 3; // BGR

static const char *SHM_LEFT_NAME = "/camera_left";
static const char *SEM_LEFT_NAME = "/camera_left_sem";
static const char *SHM_RIGHT_NAME = "/camera_right";
static const char *SEM_RIGHT_NAME = "/camera_right_sem";

#pragma pack(push, 1)
struct CameraFrameHeader
{
    uint32_t sequenceNum;
    int64_t timestampUs;
    uint32_t width;
    uint32_t height;
    uint8_t valid;
};
#pragma pack(pop)

static std::atomic<bool> g_running{true};
void signalHandler(int) { g_running = false; }

struct ShmContext
{
    uint8_t *ptr = nullptr;
    sem_t *sem = SEM_FAILED;
    size_t size = 0;
};

ShmContext openShm(const char *shmName, const char *semName)
{
    ShmContext ctx;
    ctx.size = sizeof(CameraFrameHeader) + FRAME_BYTES;

    int fd = shm_open(shmName, O_CREAT | O_RDWR, 0666);
    if (fd < 0)
        return ctx;
    if (ftruncate(fd, ctx.size) < 0)
    {
        close(fd);
        return ctx;
    }

    void *raw = mmap(nullptr, ctx.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (raw == MAP_FAILED)
        return ctx;

    ctx.ptr = static_cast<uint8_t *>(raw);
    memset(ctx.ptr, 0, ctx.size);

    ctx.sem = sem_open(semName, O_CREAT, 0666, 1);
    if (ctx.sem == SEM_FAILED)
    {
        munmap(raw, ctx.size);
        ctx.ptr = nullptr;
    }
    return ctx;
}

void closeShm(ShmContext &ctx)
{
    if (ctx.ptr)
    {
        munmap(ctx.ptr, ctx.size);
        ctx.ptr = nullptr;
    }
    if (ctx.sem != SEM_FAILED)
    {
        sem_close(ctx.sem);
        ctx.sem = SEM_FAILED;
    }
}

bool semTimedWait(sem_t *sem, int timeoutMs = 100)
{
    struct timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += timeoutMs * 1'000'000L;
    if (ts.tv_nsec >= 1'000'000'000L)
    {
        ts.tv_sec++;
        ts.tv_nsec -= 1'000'000'000L;
    }
    return sem_timedwait(sem, &ts) == 0;
}

void writeFrame(ShmContext &ctx, const cv::Mat &frame, uint32_t seq)
{
    if (!ctx.ptr || ctx.sem == SEM_FAILED)
        return;
    if (!semTimedWait(ctx.sem))
        return;

    auto *header = reinterpret_cast<CameraFrameHeader *>(ctx.ptr);
    uint8_t *pixels = ctx.ptr + sizeof(CameraFrameHeader);

    auto now = std::chrono::system_clock::now().time_since_epoch();
    header->timestampUs = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
    header->sequenceNum = seq;
    header->width = FRAME_WIDTH;
    header->height = FRAME_HEIGHT;
    header->valid = 1;
    memcpy(pixels, frame.data, FRAME_BYTES);

    sem_post(ctx.sem);
}

void runCamera(const std::string &path, ShmContext &shm, const std::string &name,
               int fps, std::atomic<bool> &running)
{
    while (running)
    {
        cv::VideoCapture cap;
        bool opened = false;
        try
        {
            int idx;
            if (std::istringstream(path) >> idx)
                cap.open(idx, cv::CAP_V4L2);
            else
                cap.open(path, cv::CAP_V4L2);
            opened = cap.isOpened();
        }
        catch (...)
        {
        }

        if (!opened)
        {
            spdlog::error("CAMERA_{}_FAILED_TO_OPEN", name);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
        cap.set(cv::CAP_PROP_FPS, fps);

        spdlog::info("CAMERA_{}_CONNECTED", name);

        cv::Mat frame;
        uint32_t seq = 0;
        auto interval = std::chrono::milliseconds(1000 / fps);

        while (running)
        {
            auto start = std::chrono::steady_clock::now();

            if (!cap.read(frame) || frame.empty())
            {
                spdlog::warn("CAMERA_{}_DISCONNECTED", name);
                break;
            }

            writeFrame(shm, frame, ++seq);

            auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed < interval)
                std::this_thread::sleep_for(interval - elapsed);
        }
    }
}

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    spdlog::info("CAMERA_STARTING");

    ShmContext leftShm = openShm(SHM_LEFT_NAME, SEM_LEFT_NAME);
    ShmContext rightShm = openShm(SHM_RIGHT_NAME, SEM_RIGHT_NAME);

    if (!leftShm.ptr || !rightShm.ptr)
    {
        spdlog::error("SHM_INIT_FAILED");
        return 1;
    }

    // IF FPS IS CHANGED, CHANGE IN CHRONOS TOO
    std::thread leftThread([&]()
                           { runCamera("/dev/video0", leftShm, "left", 12, g_running); });
    std::thread rightThread([&]()
                            { runCamera("/dev/video2", rightShm, "right", 12, g_running); });

    leftThread.join();
    rightThread.join();

    closeShm(leftShm);
    closeShm(rightShm);
    shm_unlink(SHM_LEFT_NAME);
    sem_unlink(SEM_LEFT_NAME);
    shm_unlink(SHM_RIGHT_NAME);
    sem_unlink(SEM_RIGHT_NAME);
    return 0;
}