#ifndef NUTTX_THREAD_H
#define NUTTX_THREAD_H

#include "vcu/platform/thread_interface.h"
#include <pthread.h>
#include <functional>

class NuttxThread : public ThreadInterface {
private:
    pthread_t thread_;
    bool thread_started_;

public:
    NuttxThread();
    ~NuttxThread() override;
    
    bool start(std::function<void()> func) override;
    bool join() override;
    bool detach() override;
};

#endif // NUTTX_THREAD_H
