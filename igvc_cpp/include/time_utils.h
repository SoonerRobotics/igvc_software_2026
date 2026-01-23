    #ifndef IGVC_CPP_TIME_UTILS_H
#define IGVC_CPP_TIME_UTILS_H

#pragma once

#include <ctime>
#include <cstdint>

inline int64_t GetUnixTicks()
{
    timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 10'000'000LL + ts.tv_nsec / 100;
}

#endif