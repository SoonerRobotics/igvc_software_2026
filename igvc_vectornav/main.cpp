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

#include "spdlog/spdlog.h"
#include "vectornav/Interface/Sensor.hpp"

#pragma pack(push, 1)
struct VectorNavReport {
    // GPS
    double latitude;
    double longitude;
    double altitude;
    float velNorthMs;
    float velEastMs;
    float velDownMs;
    uint8_t numSats;
    uint8_t gpsFix;

    // Attitude
    float yaw;
    float pitch;
    float roll;

    // Metadata
    int64_t timestampUs; // Unix time in microseconds
    uint32_t sequenceNum; // Increments on every write, lets reader detect new data
    uint8_t valid; // 1 = data is trustworthy
};
#pragma pack(pop)

static const char *SHM_NAME = "/vectornav_report";
static const char *SEM_NAME = "/vectornav_sem";
static const int SHM_SIZE = sizeof(VectorNavReport);

static std::atomic<bool> g_running{true};

void signalHandler(int sig) {
    spdlog::warn("RECEIVED_SIGNAL_{}", sig);
    g_running = false;
}

struct ShmContext {
    VectorNavReport *ptr = nullptr;
    sem_t *sem = SEM_FAILED;
    int fd = -1;
};

ShmContext openSharedMemory() {
    ShmContext ctx;

    ctx.fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (ctx.fd < 0) {
        spdlog::error("SHM_OPEN_FAILED_{}", strerror(errno));
        return ctx;
    }

    if (ftruncate(ctx.fd, SHM_SIZE) < 0) {
        spdlog::error("FTRUNCATE_FAILED_{}", strerror(errno));
        close(ctx.fd);
        ctx.fd = -1;
        return ctx;
    }

    void *raw = mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, ctx.fd, 0);
    close(ctx.fd); // fd no longer needed after mmap
    ctx.fd = -1;

    if (raw == MAP_FAILED) {
        spdlog::error("NMAP_FAILED_{}", strerror(errno));
        return ctx;
    }

    ctx.ptr = static_cast<VectorNavReport *>(raw);
    memset(ctx.ptr, 0, SHM_SIZE);

    ctx.sem = sem_open(SEM_NAME, O_CREAT, 0666, 1);
    if (ctx.sem == SEM_FAILED) {
        spdlog::error("SEM_OPEN_FAILED_{}", strerror(errno));
        munmap(raw, SHM_SIZE);
        ctx.ptr = nullptr;
    }

    return ctx;
}

void closeSharedMemory(ShmContext &ctx) {
    if (ctx.ptr) {
        munmap(ctx.ptr, SHM_SIZE);
        ctx.ptr = nullptr;
    }
    if (ctx.sem != SEM_FAILED) {
        sem_close(ctx.sem);
        ctx.sem = SEM_FAILED;
    }
}

void cleanupSharedMemory() {
    shm_unlink(SHM_NAME);
    sem_unlink(SEM_NAME);
}

std::optional<VN::Registers::System::BinaryOutput1> setupSensor(std::shared_ptr<VN::Sensor> sensor) {
    VN::Registers::System::BinaryOutput1 mOutput;
    mOutput.rateDivisor = 40;

    mOutput.asyncMode = VN::Registers::System::BinaryOutput::AsyncMode{};
    mOutput.asyncMode->serial1 = 1;

    // GNSS
    mOutput.gnss = VN::Registers::System::BinaryOutput::Gnss{};
    mOutput.gnss.gnss1PosLla = 1;
    mOutput.gnss.gnss1VelNed = 1;
    mOutput.gnss.gnss1Fix = 1;
    mOutput.gnss.gnss1NumSats = 1;

    // Attitude
    mOutput.common = VN::Registers::System::BinaryOutput::Common{};
    mOutput.common.ypr = 1;

    mOutput.ins = VN::Registers::System::BinaryOutput::Ins{};
    mOutput.ins.velNed = 1;

    sensor->asyncOutputEnable(VN::AsyncOutputEnable::State::Disable);

    auto cmd = mOutput.toWriteCommand();
    if (!cmd.has_value()) {
        spdlog::error("BUILD_WRITE_FAILED");
        return std::nullopt;
    }

    auto writeError = sensor->sendCommand(&cmd.value(), VN::Sensor::SendCommandBlockMode::Block);
    if (writeError != VN::Error::None) {
        spdlog::error("WRITE_OUTPUT_FAILED_", VN::errorCodeToString(writeError));
        return std::nullopt;
    }

    sensor->asyncOutputEnable(VN::AsyncOutputEnable::State::Enable);
    return mOutput;
}

void writeReport(ShmContext &ctx, const VectorNavReport &report) {
    if (!ctx.ptr || ctx.sem == SEM_FAILED) return;

    // Timeout after 100ms rather than blocking forever
    struct timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += 100'000'000L;
    if (ts.tv_nsec >= 1'000'000'000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1'000'000'000L;
    }

    if (sem_timedwait(ctx.sem, &ts) != 0) {
        spdlog::warn("SEM_TIMEDWAIT_FAILED");
        return;
    }

    memcpy(ctx.ptr, &report, SHM_SIZE);
    sem_post(ctx.sem);
}

int main() {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    spdlog::set_pattern("{\"timestamp\": \"%Y-%m-%dT%H:%M:%S.%eZ\", \"level\": \"%l\", \"name\": \"%n\", \"message\": \"%v\"}");
    spdlog::info("VECTORNAV_STARTING");

    ShmContext shm = openSharedMemory();
    if (!shm.ptr) {
        spdlog::error("SHARED_MEM_FAILED");
        return 1;
    }

    uint32_t seqNum = 0;

    while (g_running) {
        auto mSensor = std::make_shared<VN::Sensor>();

        VN::Error connectError = mSensor->autoConnect("/dev/autonav-vn");
        if (connectError != VN::Error::None) {
            spdlog::error("VECTORNAV_CONNECTION_FAILED_{}", VN::errorCodeToString(connectError));
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        spdlog::info("BAUD_RATE_{}", static_cast<uint16_t>(mSensor->connectedBaudRate().value()));

        auto reg = setupSensor(mSensor);
        if (!reg.has_value()) {
            spdlog::error("SENSOR_SETUP_FAILED");
            mSensor->disconnect();
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        while (g_running && mSensor->verifySensorConnectivity()) {
            VN::Sensor::CompositeDataQueueReturn compositeData = mSensor->getNextMeasurement();
            if (!compositeData) continue;
            if (!compositeData->matchesMessage(reg.value())) continue;

            // ── Extract fields safely ─────────────────────────────────────
            VectorNavReport report{};
            bool dataValid = true;

            if (compositeData->gnss.gnss1PosLla.has_value()) {
                report.latitude = compositeData->gnss.gnss1PosLla->lat;
                report.longitude = compositeData->gnss.gnss1PosLla->lon;
                report.altitude = compositeData->gnss.gnss1PosLla->alt;
            } else {
                spdlog::warn("GNSS_POS_MISSING");
                dataValid = false;
            }

            if (compositeData->ins.velNed.has_value()) {
                report.velNorthMs = static_cast<float>(compositeData->ins.velNed.value()[0]);  // North
                report.velEastMs  = static_cast<float>(compositeData->ins.velNed.value()[1]);  // East
                report.velDownMs  = static_cast<float>(compositeData->ins.velNed.value()[2]);  // Down
            } else {
                spdlog::warn("GNSS_VEL_NED_MISSING");
                dataValid = false;
            }

            if (compositeData->attitude.ypr.has_value()) {
                report.yaw = compositeData->attitude.ypr->yaw;
                report.pitch = compositeData->attitude.ypr->pitch;
                report.roll = compositeData->attitude.ypr->roll;
            } else {
                spdlog::warn("YPR_MISSING");
                dataValid = false;
            }

            report.numSats = compositeData->gnss.gnss1NumSats.value_or(0);
            report.gpsFix = compositeData->gnss.gnss1Fix.value_or(0);

            // Timestamp + sequence
            auto now = std::chrono::system_clock::now().time_since_epoch();
            report.timestampUs = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
            report.sequenceNum = ++seqNum;
            report.valid = dataValid ? 1 : 0;

            writeReport(shm, report);
            spdlog::info(
                "[#{}] Lat: {:.6f}, Lon: {:.6f}, Alt: {:.2f}m | "
                "Yaw: {:.2f}, Pitch: {:.2f}, Roll: {:.2f} | "
                "Vel N/E/D: {:.2f}/{:.2f}/{:.2f} m/s | "
                "Sats: {}, Fix: {}, Valid: {}",
                seqNum,
                report.latitude, report.longitude, report.altitude,
                report.yaw, report.pitch, report.roll,
                report.velNorthMs, report.velEastMs, report.velDownMs,
                report.numSats, report.gpsFix, dataValid);
        }

        spdlog::warn("VECTORNAV_DISCONNECTED");
        mSensor->disconnect();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    spdlog::info("VECTORNAV_SHUTDOWN");
    closeSharedMemory(shm);
    cleanupSharedMemory();
    return 0;
}
