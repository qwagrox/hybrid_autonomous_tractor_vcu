// Copyright 2025 Manus AI

#ifndef VCU_CONFIG_CONFIG_MANAGER_H_
#define VCU_CONFIG_CONFIG_MANAGER_H_

#include <string>
#include <vector>

namespace vcu {
namespace config {

struct VcuConfig {
    std::string can_bus_name;
    uint32_t data_timeout_ms;
    // Add other configuration parameters here
};

class ConfigManager {
public:
    virtual ~ConfigManager() = default;

    virtual bool load_config(const std::string& path) = 0;
    virtual VcuConfig get_vcu_config() const = 0;
};

} // namespace config
} // namespace vcu

#endif // VCU_CONFIG_CONFIG_MANAGER_H_
