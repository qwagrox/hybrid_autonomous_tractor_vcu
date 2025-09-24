#ifndef NUTTX_TIME_H
#define NUTTX_TIME_H

#include "vcu/platform/time_interface.h"

class NuttxTime : public TimeInterface {
public:
    NuttxTime();
    ~NuttxTime() override;
    
    uint64_t get_monotonic_time_ms() override;
    void sleep_ms(uint32_t ms) override;
};

#endif // NUTTX_TIME_H
