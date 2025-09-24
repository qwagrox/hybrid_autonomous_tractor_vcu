#ifndef LINUX_CONDITION_VARIABLE_H
#define LINUX_CONDITION_VARIABLE_H

#include "vcu/platform/condition_variable_interface.h"
#include <condition_variable>

class LinuxConditionVariable : public ConditionVariableInterface {
private:
    std::condition_variable cv_;

public:
    LinuxConditionVariable();
    ~LinuxConditionVariable() override;
    
    void wait(MutexInterface& mutex) override;
    void notify_one() override;
    void notify_all() override;
};

#endif // LINUX_CONDITION_VARIABLE_H
