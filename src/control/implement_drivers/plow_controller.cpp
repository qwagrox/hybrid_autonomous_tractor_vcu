#include "../../../../include/control/implement_drivers/plow_controller.hpp"
#include "../../../../include/integration/isobus_adapter.hpp"
#include <stdexcept>

PlowController::PlowController(std::shared_ptr<ISOBUSAdapter> isobus_adapter)
    : isobus_adapter_(isobus_adapter) {
    if (!isobus_adapter) {
        throw std::invalid_argument("ISOBUSAdapter cannot be null.");
    }
    status_.state = ImplementState::IDLE;
}

bool PlowController::initialize(const ImplementConfig& config) {
    if (config.type != getType()) {
        return false; // 类型不匹配
    }
    config_ = config;
    status_.state = ImplementState::CONFIGURED;
    status_.is_connected = true; // 假设在初始化时已连接
    return true;
}

bool PlowController::start() {
    if (status_.state != ImplementState::CONFIGURED && status_.state != ImplementState::IDLE) {
        return false; // 状态不正确
    }
    status_.state = ImplementState::ACTIVE;
    // 实际作业开始时，可能需要发送一个特定的ISOBUS指令
    // isobus_adapter_->sendEnableCommand(...);
    return true;
}

bool PlowController::stop() {
    if (status_.state != ImplementState::ACTIVE) {
        return false;
    }
    status_.state = ImplementState::CONFIGURED;
    // 发送指令将犁提升到运输位置
    sendDepthCommand(0); 
    return true;
}

bool PlowController::setWorkParameter(const std::string& key, double value) {
    if (key == "depth") {
        target_depth_ = value;
        if (status_.state == ImplementState::ACTIVE) {
            sendDepthCommand(target_depth_);
        }
        return true;
    }
    return false; // 不支持的参数
}

ImplementStatus PlowController::getStatus() const {
    return status_;
}

DiagnosticReport PlowController::runDiagnostics() {
    DiagnosticReport report;
    // 模拟诊断：检查与ISOBUS适配器的连接
    if (isobus_adapter_) {
        report.passed = true;
        report.findings.push_back("ISOBUS adapter is connected.");
    } else {
        report.passed = false;
        report.findings.push_back("ISOBUS adapter is not connected.");
    }
    return report;
}

void PlowController::update(double dt) {
    // 闭环控制逻辑可以在这里实现
    // 例如，根据传感器反馈调整深度
    // status_.current_depth = readDepthFromISOBUS();
    // if (status_.state == ImplementState::ACTIVE) {
    //     double error = target_depth_ - status_.current_depth;
    //     if (std::abs(error) > 0.01) { // 1cm 容差
    //         sendDepthCommand(target_depth_);
    //     }
    // }
}

void PlowController::sendDepthCommand(double depth) {
    // 假设PGN 0xEF00 用于控制犁的深度
    // 实际的PGN需要根据农具制造商的文档来确定
    const uint32_t PGN_PLOW_DEPTH = 0xEF00;
    isobus_adapter_->sendValueCommand(0x25, PGN_PLOW_DEPTH, depth); // 假设目标地址为0x25
}

