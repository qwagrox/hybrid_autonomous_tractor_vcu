// Copyright 2025 Manus AI

#include "vcu/config/json_config_manager.h"
#include <fstream>
#include <sstream>

namespace vcu {
namespace config {

JsonConfigManager::JsonConfigManager()
    : config_loaded_(false) {
    // Set default configuration
    config_.can_bus_name = "can0";
    config_.data_timeout_ms = 1000;
}

JsonConfigManager::~JsonConfigManager() = default;

bool JsonConfigManager::load_config(const std::string& path) {
    std::ifstream config_file(path);
    if (!config_file.is_open()) {
        // If file doesn't exist, use default configuration
        config_loaded_ = true;
        return true;
    }

    std::stringstream buffer;
    buffer << config_file.rdbuf();
    std::string json_content = buffer.str();

    // Parse JSON configuration
    parse_json_config(json_content);

    config_loaded_ = true;
    return true;
}

VcuConfig JsonConfigManager::get_vcu_config() const {
    return config_;
}

bool JsonConfigManager::parse_json_config(const std::string& json_content) {
    // Simple JSON parsing for basic configuration
    // In a real implementation, use a proper JSON library like nlohmann/json
    
    // Look for can_bus_name
    size_t can_bus_pos = json_content.find("\"can_bus_name\"");
    if (can_bus_pos != std::string::npos) {
        size_t colon_pos = json_content.find(":", can_bus_pos);
        if (colon_pos != std::string::npos) {
            size_t quote_start = json_content.find("\"", colon_pos);
            size_t quote_end = json_content.find("\"", quote_start + 1);
            if (quote_start != std::string::npos && quote_end != std::string::npos) {
                config_.can_bus_name = json_content.substr(quote_start + 1, quote_end - quote_start - 1);
            }
        }
    }

    // Look for data_timeout_ms
    size_t timeout_pos = json_content.find("\"data_timeout_ms\"");
    if (timeout_pos != std::string::npos) {
        size_t colon_pos = json_content.find(":", timeout_pos);
        if (colon_pos != std::string::npos) {
            size_t number_start = colon_pos + 1;
            while (number_start < json_content.length() && 
                   (json_content[number_start] == ' ' || json_content[number_start] == '\t')) {
                number_start++;
            }
            
            size_t number_end = number_start;
            while (number_end < json_content.length() && 
                   std::isdigit(json_content[number_end])) {
                number_end++;
            }
            
            if (number_end > number_start) {
                std::string number_str = json_content.substr(number_start, number_end - number_start);
                try {
                    config_.data_timeout_ms = std::stoul(number_str);
                } catch (const std::exception&) {
                    // Keep default value on parsing error
                }
            }
        }
    }

    return true;
}

} // namespace config
} // namespace vcu
