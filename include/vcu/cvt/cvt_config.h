#ifndef CVT_CONFIG_H
#define CVT_CONFIG_H

#include "vcu/common/vcu_types.h"
#include <string>

namespace vcu {
namespace cvt {

/**
 * @struct CvtConfig
 * @brief Configuration structure for CVT control.
 *
 * This structure contains all configuration parameters needed
 * to initialize and operate a CVT control strategy.
 */
struct CvtConfig {
    // CVT制造商类型
    common::CvtManufacturer manufacturer = common::CvtManufacturer::HMCVT_VENDOR1;
    
    // 传动比范围
    float min_ratio = -0.9f;
    float max_ratio = 2.0f;
    
    // 控制参数
    float ratio_tolerance = 0.01f;          // 传动比容差
    float adjustment_rate = 0.1f;           // 调整速率
    uint8_t control_period_ms = 10;         // 控制周期(ms)
    uint8_t status_period_ms = 100;         // 状态反馈周期(ms)
    
    // 安全参数
    float max_oil_temp = 90.0f;             // 最大油温(°C)
    float min_pressure = 18.0f;             // 最小压力(bar)
    uint16_t min_engine_rpm = 1000;         // 最小发动机转速(rpm)
    
    // 协议特定参数
    uint32_t control_can_id = 0x18FFF023;   // 控制消息CAN ID
    uint32_t status_can_id = 0x18FFF024;    // 状态消息CAN ID
    
    // 工作模式配置
    bool enable_four_wheel_drive = false;   // 启用四驱
    bool enable_differential_lock = false;  // 启用差速锁
    bool enable_auto_clutch = true;         // 启用自动离合器
    
    // 调试和诊断
    bool enable_debug_logging = false;      // 启用调试日志
    std::string log_file_path = "";         // 日志文件路径
};

/**
 * @class CvtConfigManager
 * @brief Manager class for CVT configuration.
 *
 * This class handles loading, saving, and managing CVT configuration
 * parameters from various sources (files, environment variables, etc.).
 */
class CvtConfigManager {
public:
    /**
     * @brief Loads CVT configuration from a file.
     *
     * @param config_file_path Path to the configuration file.
     * @return The loaded CVT configuration.
     */
    static CvtConfig load_from_file(const std::string& config_file_path);
    
    /**
     * @brief Loads CVT configuration from environment variables.
     *
     * @return The loaded CVT configuration.
     */
    static CvtConfig load_from_environment();
    
    /**
     * @brief Gets the default CVT configuration.
     *
     * @return The default CVT configuration.
     */
    static CvtConfig get_default_config();
    
    /**
     * @brief Validates a CVT configuration.
     *
     * @param config The configuration to validate.
     * @return True if the configuration is valid, false otherwise.
     */
    static bool validate_config(const CvtConfig& config);
    
    /**
     * @brief Saves CVT configuration to a file.
     *
     * @param config The configuration to save.
     * @param config_file_path Path to the configuration file.
     * @return True if successful, false otherwise.
     */
    static bool save_to_file(const CvtConfig& config, const std::string& config_file_path);

private:
    CvtConfigManager() = default;
};

} // namespace cvt
} // namespace vcu

#endif // CVT_CONFIG_H
