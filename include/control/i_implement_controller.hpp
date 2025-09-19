#ifndef I_IMPLEMENT_CONTROLLER_HPP
#define I_IMPLEMENT_CONTROLLER_HPP

#include <string>
#include <vector>
#include "../vcu_core_types.hpp"

enum class ImplementState : uint8_t {
    IDLE,
    ACTIVE,
    ERROR,
    CONFIGURED
};

struct ImplementConfig {
    std::string name;
    std::string type;
};

struct ImplementStatus {
    ImplementState state;
    bool is_connected;
    float current_depth;
    float current_rate;
};

struct DiagnosticReport {
    bool passed;
    std::vector<std::string> findings;
};


/**
 * @class IImplementController
 * @brief 农具控制器抽象接口
 * 
 * 定义了所有农具驱动必须实现的标准化接口，以实现插件式、可扩展的农具控制。
 */
class IImplementController {
public:
    virtual ~IImplementController() = default;

    /**
     * @brief 获取农具类型
     * @return 农具的类型标识符 (e.g., "Plow", "Seeder")
     */
    virtual std::string getType() const = 0;

    /**
     * @brief 初始化农具控制器
     * @param config 包含农具特定参数的配置结构体
     * @return 如果初始化成功，返回true；否则返回false
     */
    virtual bool initialize(const ImplementConfig& config) = 0;

    /**
     * @brief 启动农具作业
     * @return 如果成功启动，返回true
     */
    virtual bool start() = 0;

    /**
     * @brief 停止农具作业
     * @return 如果成功停止，返回true
     */
    virtual bool stop() = 0;

    /**
     * @brief 设置作业参数
     * @param key 参数名 (e.g., "depth", "rate")
     * @param value 参数值
     * @return 如果设置成功，返回true
     */
    virtual bool setWorkParameter(const std::string& key, double value) = 0;

    /**
     * @brief 获取当前农具状态
     * @return 包含当前状态信息的结构体
     */
    virtual ImplementStatus getStatus() const = 0;

    /**
     * @brief 执行诊断程序
     * @return 包含诊断结果的报告
     */
    virtual DiagnosticReport runDiagnostics() = 0;

    /**
     * @brief 更新控制逻辑（在每个控制周期调用）
     * @param dt 时间增量 (delta time)
     */
    virtual void update(double dt) = 0;
};

#endif // I_IMPLEMENT_CONTROLLER_HPP

