#include "nuttx_mutex.h"
#include <cstring>

NuttxMutex::NuttxMutex() {
    memset(&mutex_, 0, sizeof(mutex_));
    pthread_mutex_init(&mutex_, nullptr);
}

NuttxMutex::~NuttxMutex() {
    pthread_mutex_destroy(&mutex_);
}

void NuttxMutex::lock() {
    pthread_mutex_lock(&mutex_);
}

bool NuttxMutex::try_lock() {
    return pthread_mutex_trylock(&mutex_) == 0;
}

void NuttxMutex::unlock() {
    pthread_mutex_unlock(&mutex_);
}

void* NuttxMutex::get_native_handle() {
    return &mutex_;
}
