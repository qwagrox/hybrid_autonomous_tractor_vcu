#include "nuttx_platform.h"
#include "nuttx_thread.h"
#include "nuttx_mutex.h"
#include "nuttx_condition_variable.h"
#include "nuttx_time.h"
#include "nuttx_can_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

NuttxPlatform::NuttxPlatform() {}

NuttxPlatform::~NuttxPlatform() {}

std::unique_ptr<ThreadInterface> NuttxPlatform::create_thread() {
    return std::make_unique<NuttxThread>();
}

std::unique_ptr<MutexInterface> NuttxPlatform::create_mutex() {
    return std::make_unique<NuttxMutex>();
}

std::unique_ptr<ConditionVariableInterface> NuttxPlatform::create_condition_variable() {
    return std::make_unique<NuttxConditionVariable>();
}

std::unique_ptr<TimeInterface> NuttxPlatform::create_time_interface() {
    return std::make_unique<NuttxTime>();
}

std::unique_ptr<CanInterface> NuttxPlatform::create_can_interface(const std::string& interface_name) {
    return std::make_unique<NuttxCanInterface>();
}

bool NuttxPlatform::file_exists(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

std::string NuttxPlatform::read_file(const std::string& path) {
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0) {
        return "";
    }
    
    std::string content;
    char buffer[1024];
    ssize_t bytes_read;
    
    while ((bytes_read = read(fd, buffer, sizeof(buffer))) > 0) {
        content.append(buffer, bytes_read);
    }
    
    close(fd);
    return content;
}

bool NuttxPlatform::write_file(const std::string& path, const std::string& content) {
    int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        return false;
    }
    
    ssize_t bytes_written = write(fd, content.c_str(), content.length());
    close(fd);
    
    return bytes_written == static_cast<ssize_t>(content.length());
}
