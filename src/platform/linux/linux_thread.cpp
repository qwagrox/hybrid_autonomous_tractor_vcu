#include "linux_thread.h"

LinuxThread::LinuxThread() : thread_started_(false) {}

LinuxThread::~LinuxThread() {
    if (thread_started_ && thread_.joinable()) {
        thread_.join();
    }
}

bool LinuxThread::start(std::function<void()> func) {
    if (thread_started_) {
        return false; // 线程已经启动
    }
    
    try {
        thread_ = std::thread(func);
        thread_started_ = true;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool LinuxThread::join() {
    if (!thread_started_ || !thread_.joinable()) {
        return false;
    }
    
    try {
        thread_.join();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool LinuxThread::detach() {
    if (!thread_started_ || !thread_.joinable()) {
        return false;
    }
    
    try {
        thread_.detach();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}
