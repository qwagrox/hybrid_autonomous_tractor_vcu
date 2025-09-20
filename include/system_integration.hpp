#ifndef SYSTEM_INTEGRATION_HPP
#define SYSTEM_INTEGRATION_HPP

#include "vcu_core_types.hpp"
#include <memory>
#include <vector>

namespace VCUCore {

/**
 * @brief 系统集成类，负责协调各个子系统的工作
 */
class SystemIntegration {
public:
    SystemIntegration();
    ~SystemIntegration();

    /**
     * @brief 初始化系统集成
     * @return 初始化是否成功
     */
    bool initialize();

    /**
     * @brief 运行系统集成的主循环
     * @return 运行是否成功
     */
    bool run();

    /**
     * @brief 停止系统集成
     */
    void stop();

    /**
     * @brief 获取系统状态
     * @return 当前系统状态
     */
    SystemStatus getSystemStatus() const;

    /**
     * @brief 设置系统参数
     * @param params 系统参数
     */
    void setSystemParameters(const SystemParameters& params);

    /**
     * @brief 获取系统参数
     * @return 当前系统参数
     */
    SystemParameters getSystemParameters() const;

    /**
     * @brief 处理紧急停止
     */
    void emergencyStop();

    /**
     * @brief 检查系统健康状态
     * @return 系统是否健康
     */
    bool isSystemHealthy() const;

private:
    bool initialized_;
    bool running_;
    SystemStatus currentStatus_;
    SystemParameters systemParams_;

    /**
     * @brief 初始化各个子系统
     * @return 初始化是否成功
     */
    bool initializeSubsystems();

    /**
     * @brief 更新系统状态
     */
    void updateSystemStatus();

    /**
     * @brief 处理系统错误
     * @param errorCode 错误代码
     */
    void handleSystemError(int errorCode);
};

} // namespace VCUCore

#endif // SYSTEM_INTEGRATION_HPP
