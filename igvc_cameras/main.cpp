#include <iostream>
#include <atomic>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sstream>
#include <thread>
#include <chrono>

#include "spdlog/spdlog.h"
#include <opencv2/opencv.hpp>

static constexpr int FRAME_WIDTH = 640;
static constexpr int FRAME_HEIGHT = 480;
static constexpr int FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT * 3; // BGR

static const char *SHM_LEFT_NAME = "/camera_left";
static const char *SEM_LEFT_NAME = "/camera_left_sem";
static const char *SHM_RIGHT_NAME = "/camera_right";
static const char *SEM_RIGHT_NAME = "/camera_right_sem";
static const char *SHM_CMD_LEFT_NAME = "/camera_cmd_left";
static const char *SHM_CMD_RIGHT_NAME = "/camera_cmd_right";

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

#pragma pack(push, 1)
struct CameraCommandBlock
{
    uint32_t version;
    uint32_t fps;
    uint32_t width;
    uint32_t height;
    uint32_t fourcc;
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

struct CmdContext
{
    CameraCommandBlock *ptr = nullptr;
    int fd = -1;
};

CmdContext openCmdShm(const char *name)
{
    CmdContext ctx;
    ctx.fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (ctx.fd < 0)
        return ctx;

    if (ftruncate(ctx.fd, sizeof(CameraCommandBlock)) < 0)
    {
        close(ctx.fd);
        ctx.fd = -1;
        return ctx;
    }

    void *raw = mmap(nullptr, sizeof(CameraCommandBlock),
                     PROT_READ | PROT_WRITE, MAP_SHARED, ctx.fd, 0);
    if (raw == MAP_FAILED)
    {
        close(ctx.fd);
        ctx.fd = -1;
        return ctx;
    }

    ctx.ptr = static_cast<CameraCommandBlock *>(raw);

    if (ctx.ptr->fps == 0)
    {
        ctx.ptr->fps = 12;
        ctx.ptr->width = FRAME_WIDTH;
        ctx.ptr->height = FRAME_HEIGHT;
        ctx.ptr->fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    }

    return ctx;
}

void closeCmdShm(CmdContext &ctx)
{
    if (ctx.ptr)
    {
        munmap(ctx.ptr, sizeof(CameraCommandBlock));
        ctx.ptr = nullptr;
    }
    if (ctx.fd >= 0)
    {
        close(ctx.fd);
        ctx.fd = -1;
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

void runCamera(const std::string &path, ShmContext &shm, CmdContext &cmd,
               const std::string &name, std::atomic<bool> &running)
{
    while (running)
    {
        cv::VideoCapture cap;
        bool opened = false;
        try
        {
            int idx;
            std::istringstream ss(path);
            if (ss >> idx)
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

        uint32_t appliedVersion = 0;
        auto applyCmd = [&]()
        {
            if (!cmd.ptr)
                return;
            appliedVersion = cmd.ptr->version;
            int fps = cmd.ptr->fps > 0 ? static_cast<int>(cmd.ptr->fps) : 12;
            int width = cmd.ptr->width > 0 ? static_cast<int>(cmd.ptr->width) : FRAME_WIDTH;
            int height = cmd.ptr->height > 0 ? static_cast<int>(cmd.ptr->height) : FRAME_HEIGHT;
            uint32_t cc = cmd.ptr->fourcc > 0 ? cmd.ptr->fourcc
                                              : static_cast<uint32_t>(cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

            cap.set(cv::CAP_PROP_FOURCC, cc);
            cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
            cap.set(cv::CAP_PROP_FPS, fps);

            spdlog::info("CAMERA_{}_PROPS_APPLIED fps={} {}x{} fourcc={:#010x}",
                         name, fps, width, height, cc);
        };

        applyCmd();
        spdlog::info("CAMERA_{}_CONNECTED", name);

        cv::Mat frame;
        uint32_t seq = 0;

        while (running)
        {
            if (cmd.ptr && cmd.ptr->version != appliedVersion)
            {
                spdlog::info("CAMERA_{}_RESTARTING_FOR_NEW_PROPS", name);
                break;
            }

            int fps = (cmd.ptr && cmd.ptr->fps > 0) ? static_cast<int>(cmd.ptr->fps) : 12;
            auto interval = std::chrono::milliseconds(1000 / fps);
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
    CmdContext leftCmd = openCmdShm(SHM_CMD_LEFT_NAME);
    CmdContext rightCmd = openCmdShm(SHM_CMD_RIGHT_NAME);

    if (!leftShm.ptr || !rightShm.ptr || !leftCmd.ptr || !rightCmd.ptr)
    {
        spdlog::error("SHM_INIT_FAILED");
        return 1;
    }

    std::thread leftThread([&]()
                           { runCamera("/dev/video0", leftShm, leftCmd, "LEFT", g_running); });
    std::thread rightThread([&]()
                            { runCamera("/dev/video2", rightShm, rightCmd, "RIGHT", g_running); });

    leftThread.join();
    rightThread.join();

    closeShm(leftShm);
    closeShm(rightShm);
    closeCmdShm(leftCmd);
    closeCmdShm(rightCmd);

    shm_unlink(SHM_LEFT_NAME);
    sem_unlink(SEM_LEFT_NAME);
    shm_unlink(SHM_RIGHT_NAME);
    sem_unlink(SEM_RIGHT_NAME);
    shm_unlink(SHM_CMD_LEFT_NAME);
    shm_unlink(SHM_CMD_RIGHT_NAME);

    return 0;
}