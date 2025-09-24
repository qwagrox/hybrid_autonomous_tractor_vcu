#ifndef TIME_INTERFACE_H
#define TIME_INTERFACE_H

#include <cstdint>

class TimeInterface {
public:
    virtual ~TimeInterface() = default;
    virtual uint64_t get_monotonic_time_ms() = 0;
    virtual void sleep_ms(uint32_t ms) = 0;
};

#endif // TIME_INTERFACE_H

