#include "linux_platform.h"
#include "linux_thread.h"
#include "linux_mutex.h"
#include "linux_condition_variable.h"
#include "linux_time.h"
#include "vcu/can/socketcan_interface.h"
#include <fstream>
#include <sstream>

LinuxPlatform::LinuxPlatform() {}

LinuxPlatform::~LinuxPlatform() {}

std::unique_ptr<ThreadInterface> LinuxPlatform::create_thread() {
    return std::make_unique<LinuxThread>();
}

std::unique_ptr<MutexInterface> LinuxPlatform::create_mutex() {
    return std::make_unique<LinuxMutex>();
}

std::unique_ptr<ConditionVariableInterface> LinuxPlatform::create_condition_variable() {
    return std::make_unique<LinuxConditionVariable>();
}

std::unique_ptr<TimeInterface> LinuxPlatform::create_time_interface() {
    return std::make_unique<LinuxTime>();
}

std::unique_ptr<vcu::can::ICanInterface> LinuxPlatform::create_can_interface(const std::string& interface_name) {
    return std::make_unique<vcu::can::SocketCanInterface>();
}

bool LinuxPlatform::file_exists(const std::string& path) {
    std::ifstream file(path);
    return file.good();
}

std::string LinuxPlatform::read_file(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return "";
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

bool LinuxPlatform::write_file(const std::string& path, const std::string& content) {
    std::ofstream file(path);
    if (!file.is_open()) {
        return false;
    }
    
    file << content;
    return file.good();
}
