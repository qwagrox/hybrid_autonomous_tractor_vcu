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
    void set_drive_mode(common::DriveMode mode) override;
    void update(const common::PerceptionData& perception_data) override;
    common::CvtState get_current_state() const override;

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

    // ========== CAN Message Processing ==========
    void process_can_message(const can::CanFrame& frame);

private:
    // ========== CVT Related Constants ==========
    enum class GearPosition : uint8_t {
        NEUTRAL = 0,    // 00: N空档
        FORWARD = 1,    // 01: F前进档
        REVERSE = 2,    // 02: R倒档
        FORWARD_1 = 1,  // F1档位
        FORWARD_2 = 1   // F2档位 (暂不开发)
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
    static constexpr uint32_t LIFT_SETTING_CAN_ID = 0x189DFF32;
    static constexpr uint32_t LIFT_DEPTH_SETTING_CAN_ID = 0x18FED932;
    static constexpr uint32_t LIFT_STATUS_CAN_ID = 0x0CFEA123;
    static constexpr uint32_t MULTI_VALVE_CONTROL_CAN_ID = 0x189DFF32;
    static constexpr uint32_t MULTI_VALVE_LOCK_CAN_ID = 0x18FF9217;
    static constexpr uint32_t MULTI_VALVE_STATUS_CAN_ID = 0x18FF9317;
    static constexpr uint32_t PRESSURE_STATUS_CAN_ID = 0x18FF6617;
    static constexpr uint32_t TEMP_STATUS_CAN_ID = 0x18FF6817;

    // ========== CVT Control Methods ==========
    void send_cvt_control_message();
    void send_status_message();
    void send_engine_start_command();
    uint32_t get_current_time_ms() const;

    // ========== Hydraulic Control Methods ==========
    void send_hydraulic_control_message();

    // ========== CAN Message Parsing Methods ==========
    void parse_cvt_status_message(const can::CanFrame& frame);
    void parse_cvt_speed_info_message(const can::CanFrame& frame);
    void parse_pressure_status_message(const can::CanFrame& frame);
    void parse_temperature_status_message(const can::CanFrame& frame);

    // ========== CVT Member Variables ==========
    can::ICanInterface& can_interface_;
    common::DriveMode drive_mode_;
    float target_ratio_;
    mutable float current_ratio_;  // 当前传动比，用于渐进式调整
    bool control_enabled_;
    GearPosition gear_position_;
    uint8_t cvt_speed_value_;
    uint8_t clutch_percentage_;
    bool brake_enabled_;
    uint32_t last_cvt_control_send_time_;
    uint32_t last_status_send_time_;

    // ========== Hydraulic Member Variables ==========
    bool hydraulic_enabled_;
    float lift_position_;
    common::LiftMode lift_mode_;
    int8_t multi_valve_flows_[4];  // 4路液压阀流量控制 (-100 to 100)
    bool multi_valve_lock_;
    common::HydraulicState hydraulic_state_;
    bool hydraulic_ready_;
    uint32_t hydraulic_errors_;
    uint32_t last_hydraulic_control_send_time_;
};

} // namespace cvt
} // namespace vcu

#endif // HMCVT_VENDOR1_STRATEGY_H
