#include "linux_condition_variable.h"
#include "linux_mutex.h"

LinuxConditionVariable::LinuxConditionVariable() {}

LinuxConditionVariable::~LinuxConditionVariable() {}

void LinuxConditionVariable::wait(MutexInterface& mutex) {
    // 将抽象接口转换为具体的Linux实现
    LinuxMutex* linux_mutex = dynamic_cast<LinuxMutex*>(&mutex);
    if (linux_mutex) {
        std::unique_lock<std::mutex> lock(*static_cast<std::mutex*>(linux_mutex->get_native_handle()), std::adopt_lock);
        cv_.wait(lock);
        lock.release(); // 释放锁的所有权，让MutexInterface继续管理
    }
}

void LinuxConditionVariable::notify_one() {
    cv_.notify_one();
}

void LinuxConditionVariable::notify_all() {
    cv_.notify_all();
}
