#ifndef LINUX_THREAD_H
#define LINUX_THREAD_H

#include "vcu/platform/thread_interface.h"
#include <thread>
#include <functional>

class LinuxThread : public ThreadInterface {
private:
    std::thread thread_;
    bool thread_started_;

public:
    LinuxThread();
    ~LinuxThread() override;
    
    bool start(std::function<void()> func) override;
    bool join() override;
    bool detach() override;
};

#endif // LINUX_THREAD_H
