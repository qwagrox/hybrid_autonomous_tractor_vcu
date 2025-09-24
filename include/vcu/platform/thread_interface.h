#ifndef THREAD_INTERFACE_H
#define THREAD_INTERFACE_H

#include <functional>

class ThreadInterface {
public:
    virtual ~ThreadInterface() = default;
    virtual bool start(std::function<void()> func) = 0;
    virtual bool join() = 0;
    virtual bool detach() = 0;
};

#endif // THREAD_INTERFACE_H

