#ifndef NUTTX_CONDITION_VARIABLE_H
#define NUTTX_CONDITION_VARIABLE_H

#include "vcu/platform/condition_variable_interface.h"
#include <pthread.h>

class NuttxConditionVariable : public ConditionVariableInterface {
private:
    pthread_cond_t cv_;

public:
    NuttxConditionVariable();
    ~NuttxConditionVariable() override;
    
    void wait(MutexInterface& mutex) override;
    void notify_one() override;
    void notify_all() override;
};

#endif // NUTTX_CONDITION_VARIABLE_H
