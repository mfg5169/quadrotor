#pragma once

#include <time.h>

#include <stdexcept>

using namespace std;

class Clock {
public:
    Clock() {
        timespec_get(&timestruct_, TIME_UTC);
        nano_sec_offset_ = timestruct_.tv_nsec;
        sec_offset_ = timestruct_.tv_sec;
    }

    ~Clock() = default;

    double get_time() {
        timespec_get(&timestruct_, TIME_UTC);
        long nano = timestruct_.tv_nsec - nano_sec_offset_;
        time_t sec = timestruct_.tv_sec - sec_offset_;
        return static_cast<double>(sec + nano / 1000000000.0);
    }

private:
    struct timespec timestruct_;
    long nano_sec_offset_;
    time_t sec_offset_;
};
