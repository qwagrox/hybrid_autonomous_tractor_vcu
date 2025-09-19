// include/control/implement_drivers/fertilizer_controller.hpp
#pragma once
#include "control/i_implement_controller.hpp"

namespace VCUCore {

/**
 * @class FertilizerController
 * @brief 施肥机控制器
 */
class FertilizerController : public IImplementController {
private:
    double applicationRate_;  // 施肥率 (kg/ha)
    double currentDepth_;     // 当前施肥深度 (m)
    bool isApplying_;         // 是否正在施肥
    ImplementConfig config_;  // 配置信息

public:
    FertilizerController();
    ~FertilizerController() override = default;

    std::string getType() const override;
    bool initialize(const ImplementConfig& config) override;
    bool start() override;
    bool stop() override;
    bool setWorkParameter(const std::string& key, double value) override;
    ImplementStatus getStatus() const override;
    DiagnosticReport runDiagnostics() override;
    void update(double dt) override;

private:
    bool validateApplicationRate(double rate) const;
};

} // namespace VCUCore
