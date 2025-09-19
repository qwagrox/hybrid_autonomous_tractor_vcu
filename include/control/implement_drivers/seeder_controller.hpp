// include/control/implement_drivers/seeder_controller.hpp
#pragma once
#include "control/i_implement_controller.hpp"

namespace VCUCore {

/**
 * @class SeederController
 * @brief 播种机控制器
 */
class SeederController : public IImplementController {
private:
    double seedingRate_;      // 播种率 (kg/ha)
    double currentDepth_;     // 当前播种深度 (m)
    bool isSeeding_;          // 是否正在播种
    ImplementConfig config_;  // 配置信息

public:
    SeederController();
    ~SeederController() override = default;

    std::string getType() const override;
    bool initialize(const ImplementConfig& config) override;
    bool start() override;
    bool stop() override;
    bool setWorkParameter(const std::string& key, double value) override;
    ImplementStatus getStatus() const override;
    DiagnosticReport runDiagnostics() override;
    void update(double dt) override;

private:
    bool validateSeedingRate(double rate) const;
};

} // namespace VCUCore
