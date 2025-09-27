#ifndef HMCVT_VENDOR1_STRATEGY_H
#define HMCVT_VENDOR1_STRATEGY_H

#include "vcu/cvt/cvt_strategy.h"
#include "vcu/can/can_interface.h"
#include <cstdint>

namespace vcu {
namespace cvt {

/**
 * @class HMCVT_Vendor1_Strategy
 * @brief CVT control strategy implementation for HMCVT Vendor1 with hydraulic control.
 * 
 * This class implements both CVT transmission control and hydraulic system control
 * for HMCVT Vendor1 using CAN bus communication protocol.
 */
class HMCVT_Vendor1_Strategy : public CvtStrategy {
public:
    explicit HMCVT_Vendor1_Strategy(can::ICanInterface& can_interface);
    ~HMCVT_Vendor1_Strategy() override = default;

    // ========== CVT Control Implementation ==========
    void init() override;
    void set_target_ratio(float target_ratio) override;
    void update(const common::PerceptionData& perception_data) override;
    common::CvtState get_current_state() const override;
    void set_drive_mode(common::DriveMode mode) override;

    // ========== Hydraulic Control Implementation ==========
    void enable_hydraulic_control(bool enable) override;
    void set_lift_position(float position) override;
    void set_lift_mode(common::LiftMode mode) override;
    void execute_lift_action(common::LiftAction action) override;
    void set_multi_valve_flow(uint8_t valve_id, int8_t flow_percent) override;
    void set_multi_valve_lock(bool lock) override;
    common::HydraulicState get_hydraulic_state() const override;
    void execute_hydraulic_command(const common::HydraulicCommand& command) override;
    bool is_hydraulic_ready() const override;
    uint32_t get_hydraulic_errors() const override;

private:
    // ========== CVT Related Constants ==========
    enum class GearPosition : uint8_t {
        NEUTRAL = 0,    // 00: N空挡
        FORWARD = 1,    // 01: F前进挡
        REVERSE = 2,    // 02: R后退挡
        FORWARD_1 = 1,  // F1区段
        FORWARD_2 = 1   // F2区段（暂不开放）
    };

    // CVT CAN Message IDs
    static constexpr uint32_t CVT_CONTROL_CAN_ID = 0x18FFF023;
    static constexpr uint32_t CVT_ENGINE_CONTROL_CAN_ID = 0x18FFF024;
    static constexpr uint32_t CVT_STATUS_CAN_ID = 0x18FF6217;
    static constexpr uint32_t CVT_PTO_STATUS_CAN_ID = 0x18FF6117;
    static constexpr uint32_t CVT_SPEED_SETTING_CAN_ID = 0x18FF6317;

    // ========== Hydraulic Related Constants ==========
    
    // Hydraulic CAN Message IDs
    static constexpr uint32_t LIFT_CONTROL_CAN_ID = 0x14A7FFDE;
    static constexpr uint32_t LIFT_SETTING_CAN_ID = 0x189CFF32;
    static constexpr uint32_t LIFT_DEPTH_SETTING_CAN_ID = 0x18FED932;
    static constexpr uint32_t LIFT_STATUS_CAN_ID = 0x0CFFA123;
    static constexpr uint32_t MULTI_VALVE_CONTROL_CAN_ID = 0x189DFF32;
    static constexpr uint32_t MULTI_VALVE_LOCK_CAN_ID = 0x18FF9217;
    static constexpr uint32_t MULTI_VALVE_STATUS_CAN_ID = 0x18FF9A17;
    static constexpr uint32_t PRESSURE_STATUS_CAN_ID = 0x18FF6617;
    static constexpr uint32_t TEMP_STATUS_CAN_ID = 0x18FF6817;

    // Message sending periods (milliseconds)
    static constexpr uint32_t CVT_CONTROL_PERIOD_MS = 10;
    static constexpr uint32_t LIFT_CONTROL_PERIOD_MS = 100;
    static constexpr uint32_t MULTI_VALVE_CONTROL_PERIOD_MS = 100;

    // ========== CVT Control Methods ==========
    void send_cvt_control_message();
    void send_engine_start_command();
    void calculate_cvt_speed_value();
    void update_cvt_state();

    // ========== Hydraulic Control Methods ==========
    void send_lift_control_message();
    void send_lift_setting_message();
    void send_multi_valve_control_message();
    void send_multi_valve_lock_message();
    void update_hydraulic_state();
    void process_hydraulic_command(const common::HydraulicCommand& command);
    
    // ========== CAN Message Processing ==========
    void register_can_callbacks();
    void parse_cvt_status_message(const can::CanFrame& frame);
    void parse_cvt_speed_info_message(const can::CanFrame& frame);
    void parse_lift_status_message(const can::CanFrame& frame);
    void parse_multi_valve_status_message(const can::CanFrame& frame);
    void parse_pressure_status_message(const can::CanFrame& frame);
    void parse_temp_status_message(const can::CanFrame& frame);

    // ========== Utility Methods ==========
    void update_internal_state(const common::PerceptionData& perception_data);
    uint64_t get_current_time_ms() const;
    bool is_system_ready() const;
    void check_safety_conditions();
    uint8_t convert_flow_to_can_value(int8_t flow_percent) const;
    int8_t convert_can_to_flow_value(uint8_t can_value) const;

    // ========== Member Variables ==========
    
    // CAN interface and basic state
    can::ICanInterface& can_interface_;
    common::CvtState cvt_state_;
    common::DriveMode drive_mode_;
    float target_ratio_;

    // CVT specific control parameters
    bool control_enabled_;
    GearPosition gear_position_;
    int8_t cvt_speed_value_;        // -100 to 100
    uint8_t clutch_percentage_;     // 0 to 100
    bool brake_enabled_;
    bool four_wheel_drive_;
    bool differential_lock_;
    bool engine_start_;

    // Hydraulic control state
    common::HydraulicState hydraulic_state_;
    common::HydraulicCommand current_hydraulic_command_;
    bool hydraulic_control_enabled_;
    
    // Hydraulic control parameters
    common::LiftMode current_lift_mode_;
    float target_lift_position_;
    uint8_t lift_speed_setting_;
    uint8_t force_position_mix_;
    uint8_t upper_limit_setting_;
    int8_t multi_valve_flows_[4];
    bool multi_valve_locked_;

    // Time management
    uint64_t last_cvt_control_send_time_;
    uint64_t last_lift_control_send_time_;
    uint64_t last_multi_valve_control_send_time_;
    uint64_t last_status_update_time_;

    // Status feedback data
    float last_theoretical_speed_;
    float last_radar_speed_;
    uint32_t hydraulic_error_flags_;
};

} // namespace cvt
} // namespace vcu

#endif // HMCVT_VENDOR1_STRATEGY_H
