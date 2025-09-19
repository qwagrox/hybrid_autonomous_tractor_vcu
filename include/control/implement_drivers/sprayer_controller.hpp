// include/control/implement_drivers/sprayer_controller.hpp
#pragma once
#include "control/i_implement_controller.hpp"

namespace VCUCore {

/**
 * @class SprayerController
 * @brief 喷雾器控制器
 */
class SprayerController : public IImplementController {
private:
    double sprayRate_;        // 喷雾率 (L/ha)
    double sprayPressure_;    // 喷雾压力 (bar)
    bool isSpraying_;         // 是否正在喷雾
    ImplementConfig config_;  // 配置信息

public:
    SprayerController();
    ~SprayerController() override = default;

    std::string getType() const override;
    bool initialize(const ImplementConfig& config) override;
    bool start() override;
    bool stop() override;
    bool setWorkParameter(const std::string& key, double value) override;
    ImplementStatus getStatus() const override;
    DiagnosticReport runDiagnostics() override;
    void update(double dt) override;

private:
    bool validateSprayRate(double rate) const;
};

} // namespace VCUCore
