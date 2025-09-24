#ifndef NUTTX_MUTEX_H
#define NUTTX_MUTEX_H

#include "vcu/platform/mutex_interface.h"
#include <pthread.h>

class NuttxMutex : public MutexInterface {
private:
    pthread_mutex_t mutex_;

public:
    NuttxMutex();
    ~NuttxMutex() override;
    
    void lock() override;
    bool try_lock() override;
    void unlock() override;
    void* get_native_handle() override;
};

#endif // NUTTX_MUTEX_H
