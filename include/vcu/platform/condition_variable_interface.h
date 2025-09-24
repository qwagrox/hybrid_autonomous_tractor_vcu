#ifndef CONDITION_VARIABLE_INTERFACE_H
#define CONDITION_VARIABLE_INTERFACE_H

#include "mutex_interface.h"

class ConditionVariableInterface {
public:
    virtual ~ConditionVariableInterface() = default;
    virtual void wait(MutexInterface& mutex) = 0;
    virtual void notify_one() = 0;
    virtual void notify_all() = 0;
};

#endif // CONDITION_VARIABLE_INTERFACE_H

