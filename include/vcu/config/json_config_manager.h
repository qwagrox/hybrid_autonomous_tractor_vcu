#ifndef VCU_CONFIG_JSON_CONFIG_MANAGER_H_
#define VCU_CONFIG_JSON_CONFIG_MANAGER_H_

#include "vcu/config/config_manager.h"
#include <string>

namespace vcu {
namespace config {

class JsonConfigManager : public ConfigManager {
public:
    JsonConfigManager();
    ~JsonConfigManager() override;

    bool load_config(const std::string& path) override;
    VcuConfig get_vcu_config() const override;

private:
    bool parse_json_config(const std::string& json_content);
    
    VcuConfig config_;
    bool config_loaded_;
};

} // namespace config
} // namespace vcu

#endif // VCU_CONFIG_JSON_CONFIG_MANAGER_H_
