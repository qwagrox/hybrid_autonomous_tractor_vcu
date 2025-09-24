#include "nuttx_thread.h"
#include <pthread.h>
#include <cstring>

// 线程包装器结构体
struct ThreadWrapper {
    std::function<void()> func;
    
    ThreadWrapper(std::function<void()> f) : func(f) {}
};

// C风格的线程入口函数
static void* thread_entry(void* arg) {
    ThreadWrapper* wrapper = static_cast<ThreadWrapper*>(arg);
    if (wrapper) {
        wrapper->func();
        delete wrapper;
    }
    return nullptr;
}

NuttxThread::NuttxThread() : thread_started_(false) {
    memset(&thread_, 0, sizeof(thread_));
}

NuttxThread::~NuttxThread() {
    if (thread_started_) {
        pthread_join(thread_, nullptr);
    }
}

bool NuttxThread::start(std::function<void()> func) {
    if (thread_started_) {
        return false; // 线程已经启动
    }
    
    ThreadWrapper* wrapper = new ThreadWrapper(func);
    
    int result = pthread_create(&thread_, nullptr, thread_entry, wrapper);
    if (result == 0) {
        thread_started_ = true;
        return true;
    } else {
        delete wrapper;
        return false;
    }
}

bool NuttxThread::join() {
    if (!thread_started_) {
        return false;
    }
    
    int result = pthread_join(thread_, nullptr);
    return result == 0;
}

bool NuttxThread::detach() {
    if (!thread_started_) {
        return false;
    }
    
    int result = pthread_detach(thread_);
    return result == 0;
}
