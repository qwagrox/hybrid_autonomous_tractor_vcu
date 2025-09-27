#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>

#include "vcu/core/hydraulic_service.h"
#include "vcu/cvt/hmcvt_vendor1_strategy.h"
#include "vcu/can/can_interface.h"

using namespace vcu;
using namespace vcu::core;
using namespace vcu::cvt;
using namespace vcu::common;
using namespace vcu::can;

// Mock CAN Interface for testing
class MockCanInterface : public ICanInterface {
public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(void, shutdown, (), (override));
    MOCK_METHOD(bool, send_frame, (const CanFrame& frame), (override));
    MOCK_METHOD(void, register_callback, (uint32_t can_id, CanCallback callback), (override));
    MOCK_METHOD(void, unregister_callback, (uint32_t can_id), (override));
    MOCK_METHOD(bool, is_connected, (), (const, override));
    MOCK_METHOD(std::string, get_interface_name, (), (const, override));
};

// Test fixture for hydraulic control tests
class HydraulicControlTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create mock CAN interface
        mock_can_interface_ = std::make_shared<MockCanInterface>();
        
        // Set up default expectations
        EXPECT_CALL(*mock_can_interface_, initialize())
            .WillRepeatedly(::testing::Return(true));
        EXPECT_CALL(*mock_can_interface_, is_connected())
            .WillRepeatedly(::testing::Return(true));
        EXPECT_CALL(*mock_can_interface_, send_frame(::testing::_))
            .WillRepeatedly(::testing::Return(true));
        EXPECT_CALL(*mock_can_interface_, register_callback(::testing::_, ::testing::_))
            .WillRepeatedly(::testing::Return());
        
        // Create CVT strategy with mock CAN interface
        cvt_strategy_ = std::make_shared<HMCVT_Vendor1_Strategy>(*mock_can_interface_);
        
        // Create hydraulic service
        hydraulic_service_ = std::make_unique<HydraulicService>(cvt_strategy_);
    }

    void TearDown() override {
        hydraulic_service_.reset();
        cvt_strategy_.reset();
        mock_can_interface_.reset();
    }

    std::shared_ptr<MockCanInterface> mock_can_interface_;
    std::shared_ptr<HMCVT_Vendor1_Strategy> cvt_strategy_;
    std::unique_ptr<HydraulicService> hydraulic_service_;
};

// ========== Basic Functionality Tests ==========

TEST_F(HydraulicControlTest, ServiceInitialization) {
    // Test successful initialization
    EXPECT_TRUE(hydraulic_service_->initialize());
    EXPECT_TRUE(hydraulic_service_->is_hydraulic_enabled());
}

TEST_F(HydraulicControlTest, ServiceShutdown) {
    // Initialize first
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Test shutdown
    hydraulic_service_->shutdown();
    EXPECT_FALSE(hydraulic_service_->is_hydraulic_enabled());
}

TEST_F(HydraulicControlTest, EnableDisableHydraulicControl) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Test disable
    EXPECT_TRUE(hydraulic_service_->enable_hydraulic_control(false));
    EXPECT_FALSE(hydraulic_service_->is_hydraulic_enabled());
    
    // Test enable
    EXPECT_TRUE(hydraulic_service_->enable_hydraulic_control(true));
    EXPECT_TRUE(hydraulic_service_->is_hydraulic_enabled());
}

// ========== Lift Control Tests ==========

TEST_F(HydraulicControlTest, SetLiftPosition) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Test valid position
    EXPECT_TRUE(hydraulic_service_->set_lift_position(50.0f));
    EXPECT_TRUE(hydraulic_service_->set_lift_position(0.0f));
    EXPECT_TRUE(hydraulic_service_->set_lift_position(100.0f));
    
    // Test invalid positions
    EXPECT_FALSE(hydraulic_service_->set_lift_position(-10.0f));
    EXPECT_FALSE(hydraulic_service_->set_lift_position(110.0f));
}

TEST_F(HydraulicControlTest, SetLiftMode) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Test all lift modes
    EXPECT_TRUE(hydraulic_service_->set_lift_mode(LiftMode::MANUAL));
    EXPECT_TRUE(hydraulic_service_->set_lift_mode(LiftMode::AUTO_DEPTH));
    EXPECT_TRUE(hydraulic_service_->set_lift_mode(LiftMode::AUTO_LOAD));
    EXPECT_TRUE(hydraulic_service_->set_lift_mode(LiftMode::SHOCK_ABSORB));
    EXPECT_TRUE(hydraulic_service_->set_lift_mode(LiftMode::POSITION_HOLD));
}

TEST_F(HydraulicControlTest, ExecuteLiftActions) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Test all lift actions
    EXPECT_TRUE(hydraulic_service_->execute_lift_action(LiftAction::STOP));
    EXPECT_TRUE(hydraulic_service_->execute_lift_action(LiftAction::LIFT_UP));
    EXPECT_TRUE(hydraulic_service_->execute_lift_action(LiftAction::LIFT_DOWN));
    EXPECT_TRUE(hydraulic_service_->execute_lift_action(LiftAction::SHOCK_ENABLE));
    EXPECT_TRUE(hydraulic_service_->execute_lift_action(LiftAction::SHOCK_DISABLE));
}

// ========== Multi-Valve Control Tests ==========

TEST_F(HydraulicControlTest, SetMultiValveFlow) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Test valid valve IDs and flow values
    for (uint8_t valve_id = 0; valve_id < 4; ++valve_id) {
        EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(valve_id, 0));    // Neutral
        EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(valve_id, 50));   // Extend
        EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(valve_id, -50));  // Retract
        EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(valve_id, 100));  // Max extend
        EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(valve_id, -100)); // Max retract
    }
    
    // Test invalid valve IDs
    EXPECT_FALSE(hydraulic_service_->set_multi_valve_flow(4, 0));
    EXPECT_FALSE(hydraulic_service_->set_multi_valve_flow(255, 0));
    
    // Test invalid flow values
    EXPECT_FALSE(hydraulic_service_->set_multi_valve_flow(0, 101));
    EXPECT_FALSE(hydraulic_service_->set_multi_valve_flow(0, -101));
}

TEST_F(HydraulicControlTest, SetMultiValveLock) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Test lock and unlock
    EXPECT_TRUE(hydraulic_service_->set_multi_valve_lock(true));
    EXPECT_TRUE(hydraulic_service_->set_multi_valve_lock(false));
}

// ========== Hydraulic Command Tests ==========

TEST_F(HydraulicControlTest, ExecuteHydraulicCommand) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Create valid hydraulic command
    HydraulicCommand command;
    command.lift_action = LiftAction::LIFT_UP;
    command.lift_mode = LiftMode::MANUAL;
    command.target_position = 75.0f;
    command.target_depth = 50.0f;
    command.lift_speed = 80;
    command.force_position_mix = 60;
    command.upper_limit = 90;
    command.valve_flows[0] = 25;
    command.valve_flows[1] = -30;
    command.valve_flows[2] = 0;
    command.valve_flows[3] = 45;
    command.valve_lock = false;
    command.hydraulic_enable = true;
    command.emergency_stop = false;
    
    EXPECT_TRUE(hydraulic_service_->execute_hydraulic_command(command));
}

TEST_F(HydraulicControlTest, ExecuteInvalidHydraulicCommand) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Create invalid hydraulic command (invalid position)
    HydraulicCommand command;
    command.target_position = 150.0f; // Invalid position > 100%
    
    EXPECT_FALSE(hydraulic_service_->execute_hydraulic_command(command));
    
    // Invalid lift speed
    command.target_position = 50.0f;
    command.lift_speed = 150; // Invalid speed > 100
    
    EXPECT_FALSE(hydraulic_service_->execute_hydraulic_command(command));
    
    // Invalid valve flow
    command.lift_speed = 50;
    command.valve_flows[0] = 150; // Invalid flow > 100
    
    EXPECT_FALSE(hydraulic_service_->execute_hydraulic_command(command));
}

// ========== Safety Tests ==========

TEST_F(HydraulicControlTest, EmergencyStop) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Set some valve flows first
    EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(0, 50));
    EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(1, -30));
    
    // Execute emergency stop
    EXPECT_TRUE(hydraulic_service_->emergency_stop());
    
    // Verify that emergency stop was executed
    // (In a real implementation, we would check that all flows are set to 0
    // and valves are locked, but this requires more sophisticated mocking)
}

TEST_F(HydraulicControlTest, OperationWithoutInitialization) {
    // Try to operate without initialization
    EXPECT_FALSE(hydraulic_service_->set_lift_position(50.0f));
    EXPECT_FALSE(hydraulic_service_->execute_lift_action(LiftAction::LIFT_UP));
    EXPECT_FALSE(hydraulic_service_->set_multi_valve_flow(0, 25));
    EXPECT_FALSE(hydraulic_service_->is_hydraulic_enabled());
}

TEST_F(HydraulicControlTest, OperationWithDisabledHydraulics) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Disable hydraulics
    EXPECT_TRUE(hydraulic_service_->enable_hydraulic_control(false));
    
    // Try to operate with disabled hydraulics
    EXPECT_FALSE(hydraulic_service_->set_lift_position(50.0f));
    EXPECT_FALSE(hydraulic_service_->execute_lift_action(LiftAction::LIFT_UP));
    EXPECT_FALSE(hydraulic_service_->set_multi_valve_flow(0, 25));
}

// ========== State Monitoring Tests ==========

TEST_F(HydraulicControlTest, GetHydraulicState) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Get hydraulic state
    HydraulicState state = hydraulic_service_->get_hydraulic_state();
    
    // Verify initial state
    EXPECT_EQ(state.lift_mode, LiftMode::MANUAL);
    EXPECT_FALSE(state.lift_moving);
    EXPECT_FALSE(state.shock_absorb_active);
    EXPECT_FALSE(state.valve_locked);
    
    // Verify valve states are initialized to neutral
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(state.valve_states[i], MultiValveState::NEUTRAL);
    }
}

TEST_F(HydraulicControlTest, GetHydraulicErrors) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Initially should have no errors
    uint32_t errors = hydraulic_service_->get_hydraulic_errors();
    EXPECT_EQ(errors, 0);
}

TEST_F(HydraulicControlTest, ResetHydraulicErrors) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Reset errors
    EXPECT_TRUE(hydraulic_service_->reset_hydraulic_errors());
}

// ========== Callback Tests ==========

TEST_F(HydraulicControlTest, StateCallbacks) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    bool callback_called = false;
    HydraulicState received_state;
    
    // Register state callback
    hydraulic_service_->register_state_callback(
        [&callback_called, &received_state](const HydraulicState& state) {
            callback_called = true;
            received_state = state;
        });
    
    // Trigger state change by updating the service
    PerceptionData perception_data;
    hydraulic_service_->update(perception_data);
    
    // Note: In a real test, we would need to simulate CAN messages
    // to trigger actual state changes. This is a simplified test.
}

TEST_F(HydraulicControlTest, ErrorCallbacks) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    bool callback_called = false;
    uint32_t received_errors = 0;
    
    // Register error callback
    hydraulic_service_->register_error_callback(
        [&callback_called, &received_errors](uint32_t errors) {
            callback_called = true;
            received_errors = errors;
        });
    
    // Trigger error condition (this would require more sophisticated mocking)
    PerceptionData perception_data;
    hydraulic_service_->update(perception_data);
}

// ========== Integration Tests ==========

TEST_F(HydraulicControlTest, CompleteWorkflowTest) {
    // Test a complete hydraulic control workflow
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // 1. Set lift mode to manual
    EXPECT_TRUE(hydraulic_service_->set_lift_mode(LiftMode::MANUAL));
    
    // 2. Lift up to 50%
    EXPECT_TRUE(hydraulic_service_->set_lift_position(50.0f));
    EXPECT_TRUE(hydraulic_service_->execute_lift_action(LiftAction::LIFT_UP));
    
    // 3. Set multi-valve flows for implement control
    EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(0, 30));  // Extend valve 0
    EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(1, -25)); // Retract valve 1
    
    // 4. Switch to auto depth mode
    EXPECT_TRUE(hydraulic_service_->set_lift_mode(LiftMode::AUTO_DEPTH));
    
    // 5. Stop all operations
    EXPECT_TRUE(hydraulic_service_->execute_lift_action(LiftAction::STOP));
    EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(0, 0));
    EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(1, 0));
    
    // 6. Lock valves for safety
    EXPECT_TRUE(hydraulic_service_->set_multi_valve_lock(true));
}

// ========== Performance Tests ==========

TEST_F(HydraulicControlTest, PerformanceTest) {
    ASSERT_TRUE(hydraulic_service_->initialize());
    
    // Measure time for multiple operations
    auto start_time = std::chrono::high_resolution_clock::now();
    
    const int num_operations = 1000;
    for (int i = 0; i < num_operations; ++i) {
        hydraulic_service_->set_lift_position(static_cast<float>(i % 100));
        hydraulic_service_->set_multi_valve_flow(i % 4, (i % 200) - 100);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time);
    
    // Verify that operations complete within reasonable time
    // (This is a basic performance check)
    EXPECT_LT(duration.count(), 100000); // Less than 100ms for 1000 operations
}

// ========== Main Test Runner ==========

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
