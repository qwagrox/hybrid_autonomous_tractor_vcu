#include "linux_mutex.h"

LinuxMutex::LinuxMutex() {}

LinuxMutex::~LinuxMutex() {}

void LinuxMutex::lock() {
    mutex_.lock();
}

bool LinuxMutex::try_lock() {
    return mutex_.try_lock();
}

void LinuxMutex::unlock() {
    mutex_.unlock();
}

void* LinuxMutex::get_native_handle() {
    return mutex_.native_handle();
}
