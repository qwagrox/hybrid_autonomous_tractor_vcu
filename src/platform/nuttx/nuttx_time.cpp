#include "nuttx_time.h"
#include <time.h>
#include <unistd.h>

NuttxTime::NuttxTime() {}

NuttxTime::~NuttxTime() {}

uint64_t NuttxTime::get_monotonic_time_ms() {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
        return static_cast<uint64_t>(ts.tv_sec) * 1000 + 
               static_cast<uint64_t>(ts.tv_nsec) / 1000000;
    }
    return 0;
}

void NuttxTime::sleep_ms(uint32_t ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&ts, nullptr);
}
