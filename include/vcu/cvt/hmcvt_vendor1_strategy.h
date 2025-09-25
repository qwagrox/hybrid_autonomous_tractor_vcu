#ifndef HMCVT_VENDOR1_STRATEGY_H
#define HMCVT_VENDOR1_STRATEGY_H

#include "vcu/cvt/cvt_strategy.h"
#include "vcu/can/can_interface.h"
#include <cstdint>

namespace vcu {
namespace cvt {

class HMCVT_Vendor1_Strategy : public CvtStrategy {
public:
    explicit HMCVT_Vendor1_Strategy(can::CanInterface& can_interface);
    ~HMCVT_Vendor1_Strategy() override = default;

    void init() override;
    void set_target_ratio(float target_ratio) override;
    void update(const common::PerceptionData& perception_data) override;
    common::CvtState get_current_state() const override;
    void set_drive_mode(common::DriveMode mode) override;

private:
    enum class GearPosition : uint8_t {
        NEUTRAL = 0,    // 00: N空挡
        FORWARD = 1,    // 01: F前进挡
        REVERSE = 2,    // 02: R后退挡
        FORWARD_1 = 1,  // F1区段
        FORWARD_2 = 1   // F2区段（暂不开放）
    };

    void send_control_message();
    void send_engine_start_command();
    void calculate_cvt_speed_value();
    void update_internal_state(const common::PerceptionData& perception_data);
    void update_cvt_state();
    void register_can_callbacks();
    void parse_status_message(const can::CanFrame& frame);
    void parse_speed_info_message(const can::CanFrame& frame);
    uint64_t get_current_time_ms() const;

    // CAN接口和基本状态
    can::CanInterface& can_interface_;
    common::CvtState cvt_state_;
    common::DriveMode drive_mode_;
    float target_ratio_;

    // HMCVT_Vendor1特定控制参数
    bool control_enabled_;
    GearPosition gear_position_;
    int8_t cvt_speed_value_;        // -100 to 100
    uint8_t clutch_percentage_;     // 0 to 100
    bool brake_enabled_;
    bool four_wheel_drive_;
    bool differential_lock_;
    bool engine_start_;

    // 时间管理
    uint64_t last_control_send_time_;
    uint64_t last_status_update_time_;

    // 状态反馈数据
    float last_theoretical_speed_;
    float last_radar_speed_;
};

} // namespace cvt
} // namespace vcu

#endif // HMCVT_VENDOR1_STRATEGY_H

