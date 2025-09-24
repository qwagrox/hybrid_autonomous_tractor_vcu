#include "linux_time.h"
#include <chrono>
#include <thread>

LinuxTime::LinuxTime() {}

LinuxTime::~LinuxTime() {}

uint64_t LinuxTime::get_monotonic_time_ms() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

void LinuxTime::sleep_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
