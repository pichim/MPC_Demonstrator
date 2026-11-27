#pragma once

#include <cerrno>
#include <system_error>
#include <time.h>

inline long long monotonic_ns()
{
    timespec time{};
    if (clock_gettime(CLOCK_MONOTONIC, &time) < 0)
        throw std::system_error(errno, std::generic_category(), "clock_gettime");
    return time.tv_sec * 1'000'000'000LL + time.tv_nsec;
}
