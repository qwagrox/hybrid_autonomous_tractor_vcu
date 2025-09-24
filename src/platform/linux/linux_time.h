#ifndef LINUX_TIME_H
#define LINUX_TIME_H

#include "vcu/platform/time_interface.h"

class LinuxTime : public TimeInterface {
public:
    LinuxTime();
    ~LinuxTime() override;
    
    uint64_t get_monotonic_time_ms() override;
    void sleep_ms(uint32_t ms) override;
};

#endif // LINUX_TIME_H
