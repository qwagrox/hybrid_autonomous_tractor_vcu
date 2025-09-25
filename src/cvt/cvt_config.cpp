#include "vcu/cvt/cvt_config.h"
#include <fstream>
#include <cstdlib>
#include <iostream>

namespace vcu {
namespace cvt {

CvtConfig CvtConfigManager::load_from_file(const std::string& config_file_path) {
    CvtConfig config = get_default_config();
    
    std::ifstream file(config_file_path);
    if (!file.is_open()) {
        std::cerr << "Warning: Could not open CVT config file: " << config_file_path << std::endl;
        return config;
    }
    
    // 简单的键值对解析（实际项目中可能使用JSON或XML）
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue; // 跳过空行和注释
        }
        
        size_t pos = line.find('=');
        if (pos == std::string::npos) {
            continue;
        }
        
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        
        // 解析配置参数
        if (key == "manufacturer") {
            if (value == "HMCVT_VENDOR1") {
                config.manufacturer = common::CvtManufacturer::HMCVT_VENDOR1;
            } else if (value == "BOSCH") {
                config.manufacturer = common::CvtManufacturer::BOSCH;
            } else if (value == "ZF") {
                config.manufacturer = common::CvtManufacturer::ZF;
            }
        } else if (key == "min_ratio") {
            config.min_ratio = std::stof(value);
        } else if (key == "max_ratio") {
            config.max_ratio = std::stof(value);
        } else if (key == "ratio_tolerance") {
            config.ratio_tolerance = std::stof(value);
        } else if (key == "adjustment_rate") {
            config.adjustment_rate = std::stof(value);
        } else if (key == "control_period_ms") {
            config.control_period_ms = static_cast<uint8_t>(std::stoi(value));
        } else if (key == "status_period_ms") {
            config.status_period_ms = static_cast<uint8_t>(std::stoi(value));
        } else if (key == "max_oil_temp") {
            config.max_oil_temp = std::stof(value);
        } else if (key == "min_pressure") {
            config.min_pressure = std::stof(value);
        } else if (key == "min_engine_rpm") {
            config.min_engine_rpm = static_cast<uint16_t>(std::stoi(value));
        } else if (key == "control_can_id") {
            config.control_can_id = static_cast<uint32_t>(std::stoul(value, nullptr, 16));
        } else if (key == "status_can_id") {
            config.status_can_id = static_cast<uint32_t>(std::stoul(value, nullptr, 16));
        } else if (key == "enable_four_wheel_drive") {
            config.enable_four_wheel_drive = (value == "true" || value == "1");
        } else if (key == "enable_differential_lock") {
            config.enable_differential_lock = (value == "true" || value == "1");
        } else if (key == "enable_auto_clutch") {
            config.enable_auto_clutch = (value == "true" || value == "1");
        } else if (key == "enable_debug_logging") {
            config.enable_debug_logging = (value == "true" || value == "1");
        } else if (key == "log_file_path") {
            config.log_file_path = value;
        }
    }
    
    file.close();
    
    if (!validate_config(config)) {
        std::cerr << "Warning: Invalid CVT configuration loaded, using defaults" << std::endl;
        return get_default_config();
    }
    
    return config;
}

CvtConfig CvtConfigManager::load_from_environment() {
    CvtConfig config = get_default_config();
    
    // 从环境变量读取配置
    const char* manufacturer_env = std::getenv("VCU_CVT_MANUFACTURER");
    if (manufacturer_env) {
        std::string manufacturer_str(manufacturer_env);
        if (manufacturer_str == "HMCVT_VENDOR1") {
            config.manufacturer = common::CvtManufacturer::HMCVT_VENDOR1;
        } else if (manufacturer_str == "BOSCH") {
            config.manufacturer = common::CvtManufacturer::BOSCH;
        } else if (manufacturer_str == "ZF") {
            config.manufacturer = common::CvtManufacturer::ZF;
        }
    }
    
    const char* min_ratio_env = std::getenv("VCU_CVT_MIN_RATIO");
    if (min_ratio_env) {
        config.min_ratio = std::stof(min_ratio_env);
    }
    
    const char* max_ratio_env = std::getenv("VCU_CVT_MAX_RATIO");
    if (max_ratio_env) {
        config.max_ratio = std::stof(max_ratio_env);
    }
    
    const char* debug_env = std::getenv("VCU_CVT_DEBUG");
    if (debug_env) {
        config.enable_debug_logging = (std::string(debug_env) == "true" || std::string(debug_env) == "1");
    }
    
    const char* log_path_env = std::getenv("VCU_CVT_LOG_PATH");
    if (log_path_env) {
        config.log_file_path = log_path_env;
    }
    
    if (!validate_config(config)) {
        std::cerr << "Warning: Invalid CVT configuration from environment, using defaults" << std::endl;
        return get_default_config();
    }
    
    return config;
}

CvtConfig CvtConfigManager::get_default_config() {
    CvtConfig config;
    
    // 使用结构体初始化时的默认值
    // 这些值已经在头文件中定义
    
    return config;
}

bool CvtConfigManager::validate_config(const CvtConfig& config) {
    // 验证传动比范围
    if (config.min_ratio >= config.max_ratio) {
        std::cerr << "Error: Invalid ratio range: min_ratio >= max_ratio" << std::endl;
        return false;
    }
    
    if (config.min_ratio < -2.0f || config.max_ratio > 3.0f) {
        std::cerr << "Error: Ratio range out of bounds" << std::endl;
        return false;
    }
    
    // 验证控制参数
    if (config.ratio_tolerance <= 0.0f || config.ratio_tolerance > 0.1f) {
        std::cerr << "Error: Invalid ratio tolerance" << std::endl;
        return false;
    }
    
    if (config.adjustment_rate <= 0.0f || config.adjustment_rate > 1.0f) {
        std::cerr << "Error: Invalid adjustment rate" << std::endl;
        return false;
    }
    
    // 验证周期参数
    if (config.control_period_ms == 0 || config.control_period_ms > 100) {
        std::cerr << "Error: Invalid control period" << std::endl;
        return false;
    }
    
    if (config.status_period_ms == 0 || config.status_period_ms > 1000) {
        std::cerr << "Error: Invalid status period" << std::endl;
        return false;
    }
    
    // 验证安全参数
    if (config.max_oil_temp < 50.0f || config.max_oil_temp > 150.0f) {
        std::cerr << "Error: Invalid max oil temperature" << std::endl;
        return false;
    }
    
    if (config.min_pressure < 5.0f || config.min_pressure > 50.0f) {
        std::cerr << "Error: Invalid min pressure" << std::endl;
        return false;
    }
    
    if (config.min_engine_rpm < 500 || config.min_engine_rpm > 3000) {
        std::cerr << "Error: Invalid min engine RPM" << std::endl;
        return false;
    }
    
    // 验证CAN ID
    if (config.control_can_id == 0 || config.status_can_id == 0) {
        std::cerr << "Error: Invalid CAN IDs" << std::endl;
        return false;
    }
    
    return true;
}

bool CvtConfigManager::save_to_file(const CvtConfig& config, const std::string& config_file_path) {
    if (!validate_config(config)) {
        std::cerr << "Error: Cannot save invalid CVT configuration" << std::endl;
        return false;
    }
    
    std::ofstream file(config_file_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open CVT config file for writing: " << config_file_path << std::endl;
        return false;
    }
    
    file << "# CVT Configuration File\n";
    file << "# Generated automatically\n\n";
    
    // 写入制造商
    file << "manufacturer=";
    switch (config.manufacturer) {
        case common::CvtManufacturer::HMCVT_VENDOR1:
            file << "HMCVT_VENDOR1";
            break;
        case common::CvtManufacturer::BOSCH:
            file << "BOSCH";
            break;
        case common::CvtManufacturer::ZF:
            file << "ZF";
            break;
        default:
            file << "UNKNOWN";
            break;
    }
    file << "\n\n";
    
    // 写入传动比参数
    file << "# Transmission ratio parameters\n";
    file << "min_ratio=" << config.min_ratio << "\n";
    file << "max_ratio=" << config.max_ratio << "\n";
    file << "ratio_tolerance=" << config.ratio_tolerance << "\n";
    file << "adjustment_rate=" << config.adjustment_rate << "\n\n";
    
    // 写入控制参数
    file << "# Control parameters\n";
    file << "control_period_ms=" << static_cast<int>(config.control_period_ms) << "\n";
    file << "status_period_ms=" << static_cast<int>(config.status_period_ms) << "\n\n";
    
    // 写入安全参数
    file << "# Safety parameters\n";
    file << "max_oil_temp=" << config.max_oil_temp << "\n";
    file << "min_pressure=" << config.min_pressure << "\n";
    file << "min_engine_rpm=" << config.min_engine_rpm << "\n\n";
    
    // 写入CAN参数
    file << "# CAN parameters (hex format)\n";
    file << "control_can_id=" << std::hex << config.control_can_id << std::dec << "\n";
    file << "status_can_id=" << std::hex << config.status_can_id << std::dec << "\n\n";
    
    // 写入功能开关
    file << "# Feature switches\n";
    file << "enable_four_wheel_drive=" << (config.enable_four_wheel_drive ? "true" : "false") << "\n";
    file << "enable_differential_lock=" << (config.enable_differential_lock ? "true" : "false") << "\n";
    file << "enable_auto_clutch=" << (config.enable_auto_clutch ? "true" : "false") << "\n\n";
    
    // 写入调试参数
    file << "# Debug parameters\n";
    file << "enable_debug_logging=" << (config.enable_debug_logging ? "true" : "false") << "\n";
    file << "log_file_path=" << config.log_file_path << "\n";
    
    file.close();
    return true;
}

} // namespace cvt
} // namespace vcu
