#ifndef PLOW_CONTROLLER_HPP
#define PLOW_CONTROLLER_HPP

#include "../i_implement_controller.hpp"
#include "../../vcu_core_types.hpp"
#include <memory>

// 前向声明
class ISOBUSAdapter;

class PlowController : public IImplementController {
public:
    explicit PlowController(std::shared_ptr<ISOBUSAdapter> isobus_adapter);
    virtual ~PlowController() = default;

    std::string getType() const override { return "Plow"; }

    bool initialize(const ImplementConfig& config) override;
    bool start() override;
    bool stop() override;
    bool setWorkParameter(const std::string& key, double value) override;
    ImplementStatus getStatus() const override;
    DiagnosticReport runDiagnostics() override;
    void update(double dt) override;

private:
    std::shared_ptr<ISOBUSAdapter> isobus_adapter_;
    ImplementStatus status_;
    ImplementConfig config_;
    double target_depth_ = 0.0;

    void sendDepthCommand(double depth);
};

#endif // PLOW_CONTROLLER_HPP

