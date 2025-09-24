#ifndef LINUX_MUTEX_H
#define LINUX_MUTEX_H

#include "vcu/platform/mutex_interface.h"
#include <mutex>

class LinuxMutex : public MutexInterface {
private:
    std::mutex mutex_;

public:
    LinuxMutex();
    ~LinuxMutex() override;
    
    void lock() override;
    bool try_lock() override;
    void unlock() override;
    void* get_native_handle() override;
};

#endif // LINUX_MUTEX_H
