#ifndef PLATFORM_INTERFACE_H
#define PLATFORM_INTERFACE_H

#include <memory>
#include <string>
#include <functional>

// 前向声明
class ThreadInterface;
class MutexInterface;
class ConditionVariableInterface;
class TimeInterface;
namespace vcu { namespace can { class ICanInterface; } }

class PlatformInterface {
public:
    virtual ~PlatformInterface() = default;

    // 线程相关
    virtual std::unique_ptr<ThreadInterface> create_thread() = 0;

    // 同步原语
    virtual std::unique_ptr<MutexInterface> create_mutex() = 0;
    virtual std::unique_ptr<ConditionVariableInterface> create_condition_variable() = 0;

    // 时间相关
    virtual std::unique_ptr<TimeInterface> create_time_interface() = 0;

    // CAN总线
    virtual std::unique_ptr<vcu::can::ICanInterface> create_can_interface(const std::string& interface_name) = 0;

    // 文件系统
    virtual bool file_exists(const std::string& path) = 0;
    virtual std::string read_file(const std::string& path) = 0;
    virtual bool write_file(const std::string& path, const std::string& content) = 0;
};

#endif // PLATFORM_INTERFACE_H

