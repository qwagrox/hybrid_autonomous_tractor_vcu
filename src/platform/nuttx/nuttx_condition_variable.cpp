#include "nuttx_condition_variable.h"
#include "nuttx_mutex.h"
#include <cstring>

NuttxConditionVariable::NuttxConditionVariable() {
    memset(&cv_, 0, sizeof(cv_));
    pthread_cond_init(&cv_, nullptr);
}

NuttxConditionVariable::~NuttxConditionVariable() {
    pthread_cond_destroy(&cv_);
}

void NuttxConditionVariable::wait(MutexInterface& mutex) {
    // 将抽象接口转换为具体的NuttX实现
    NuttxMutex* nuttx_mutex = dynamic_cast<NuttxMutex*>(&mutex);
    if (nuttx_mutex) {
        pthread_mutex_t* native_mutex = static_cast<pthread_mutex_t*>(nuttx_mutex->get_native_handle());
        pthread_cond_wait(&cv_, native_mutex);
    }
}

void NuttxConditionVariable::notify_one() {
    pthread_cond_signal(&cv_);
}

void NuttxConditionVariable::notify_all() {
    pthread_cond_broadcast(&cv_);
}
