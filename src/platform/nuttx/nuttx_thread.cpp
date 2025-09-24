#include "nuttx_thread.h"
#include <pthread.h>

namespace vcu {
namespace platform {
namespace nuttx {

struct ThreadWrapper {
    std::function<void()> func;
    
    // 修复：添加explicit关键字防止隐式转换
    explicit ThreadWrapper(std::function<void()> f) : func(f) {}
};

static void* thread_entry_point(void* arg) {
    auto* wrapper = static_cast<ThreadWrapper*>(arg);
    wrapper->func();
    delete wrapper;
    return nullptr;
}

NuttxThread::NuttxThread() : thread_id_(0), is_running_(false) {}

NuttxThread::~NuttxThread() {
    if (is_running_) {
        join();
    }
}

bool NuttxThread::start(std::function<void()> func) {
    if (is_running_) {
        return false;
    }

    auto* wrapper = new ThreadWrapper(func);
    
    int result = pthread_create(&thread_id_, nullptr, thread_entry_point, wrapper);
    if (result != 0) {
        delete wrapper;
        return false;
    }

    is_running_ = true;
    return true;
}

bool NuttxThread::join() {
    if (!is_running_) {
        return false;
    }

    int result = pthread_join(thread_id_, nullptr);
    is_running_ = false;
    return result == 0;
}

bool NuttxThread::detach() {
    if (!is_running_) {
        return false;
    }

    int result = pthread_detach(thread_id_);
    is_running_ = false;
    return result == 0;
}

bool NuttxThread::is_running() const {
    return is_running_;
}

} // namespace nuttx
} // namespace platform
} // namespace vcu
