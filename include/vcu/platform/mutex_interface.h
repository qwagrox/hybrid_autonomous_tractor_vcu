#ifndef MUTEX_INTERFACE_H
#define MUTEX_INTERFACE_H

class MutexInterface {
public:
    virtual ~MutexInterface() = default;
    virtual void lock() = 0;
    virtual bool try_lock() = 0;
    virtual void unlock() = 0;
    virtual void* get_native_handle() = 0;
};

#endif // MUTEX_INTERFACE_H

