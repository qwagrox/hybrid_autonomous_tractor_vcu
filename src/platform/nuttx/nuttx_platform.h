#ifndef NUTTX_PLATFORM_H
#define NUTTX_PLATFORM_H

#include "vcu/platform/platform_interface.h"

class NuttxPlatform : public PlatformInterface {
public:
    NuttxPlatform();
    ~NuttxPlatform() override;
    
    // 线程相关
    std::unique_ptr<ThreadInterface> create_thread() override;
    
    // 同步原语
    std::unique_ptr<MutexInterface> create_mutex() override;
    std::unique_ptr<ConditionVariableInterface> create_condition_variable() override;
    
    // 时间相关
    std::unique_ptr<TimeInterface> create_time_interface() override;
    
    // CAN总线
    std::unique_ptr<CanInterface> create_can_interface(const std::string& interface_name) override;
    
    // 文件系统
    bool file_exists(const std::string& path) override;
    std::string read_file(const std::string& path) override;
    bool write_file(const std::string& path, const std::string& content) override;
};

#endif // NUTTX_PLATFORM_H
