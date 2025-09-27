/**
 * @file test_hydraulic_control.cpp
 * @brief Simple test for hydraulic control functionality
 */

#include <gtest/gtest.h>
#include "vcu/core/hydraulic_service.h"
#include "vcu/cvt/cvt_strategy.h"
#include "vcu/common/vcu_data_types.h"

namespace vcu {
namespace core {

// Mock CVT strategy for testing
class MockCvtStrategy : public cvt::CvtStrategy {
public:
    MockCvtStrategy() = default;
    ~MockCvtStrategy() = default;

    // CVT control methods (required pure virtual functions)
    void init() override {}
    void set_target_ratio(float /* target_ratio */) override {}
    void update(const common::PerceptionData& /* perception_data */) override {}
    common::CvtState get_current_state() const override { return {}; }
    
    // Additional CVT methods (if they exist in base class)
    void set_drive_mode(common::DriveMode /* mode */) override {}

    // Hydraulic control methods
    void enable_hydraulic_control(bool /* enable */) override {}
    void set_lift_position(float /* position */) override {}
    void set_lift_mode(common::LiftMode /* mode */) override {}
    void execute_lift_action(common::LiftAction /* action */) override {}
    void set_multi_valve_flow(uint8_t /* valve_id */, int8_t /* flow_percent */) override {}
    void set_multi_valve_lock(bool /* lock */) override {}
    void execute_hydraulic_command(const common::HydraulicCommand& /* command */) override {}
    
    common::HydraulicState get_hydraulic_state() const override {
        common::HydraulicState state = {};
        state.pressure = 15.0f;
        state.temperature = 60.0f;
        state.is_ready = true;
        state.error_code = 0;
        return state;
    }
    
    // Additional required pure virtual functions
    bool is_hydraulic_ready() const override { return true; }
    uint32_t get_hydraulic_errors() const override { return 0; }
};

class HydraulicServiceTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_cvt_strategy_ = std::make_shared<MockCvtStrategy>();
        hydraulic_service_ = std::make_unique<HydraulicService>(mock_cvt_strategy_);
    }

    void TearDown() override {
        hydraulic_service_.reset();
        mock_cvt_strategy_.reset();
    }

    std::shared_ptr<MockCvtStrategy> mock_cvt_strategy_;
    std::unique_ptr<HydraulicService> hydraulic_service_;
};

TEST_F(HydraulicServiceTest, InitializeService) {
    EXPECT_TRUE(hydraulic_service_->initialize());
    EXPECT_TRUE(hydraulic_service_->is_hydraulic_ready());
}

TEST_F(HydraulicServiceTest, GetHydraulicState) {
    hydraulic_service_->initialize();
    
    auto state = hydraulic_service_->get_hydraulic_state();
    EXPECT_GT(state.pressure, 0.0f);
    EXPECT_GT(state.temperature, 0.0f);
    EXPECT_TRUE(state.is_ready);
}

TEST_F(HydraulicServiceTest, EnableHydraulicControl) {
    hydraulic_service_->initialize();
    
    EXPECT_TRUE(hydraulic_service_->enable_hydraulic_control(true));
    EXPECT_TRUE(hydraulic_service_->is_hydraulic_enabled());
    
    EXPECT_TRUE(hydraulic_service_->enable_hydraulic_control(false));
    EXPECT_FALSE(hydraulic_service_->is_hydraulic_enabled());
}

TEST_F(HydraulicServiceTest, SetLiftPosition) {
    hydraulic_service_->initialize();
    hydraulic_service_->enable_hydraulic_control(true);
    
    EXPECT_TRUE(hydraulic_service_->set_lift_position(50.0f));
    EXPECT_TRUE(hydraulic_service_->set_lift_position(0.0f));
    EXPECT_TRUE(hydraulic_service_->set_lift_position(100.0f));
}

TEST_F(HydraulicServiceTest, ExecuteLiftAction) {
    hydraulic_service_->initialize();
    hydraulic_service_->enable_hydraulic_control(true);
    
    EXPECT_TRUE(hydraulic_service_->execute_lift_action(common::LiftAction::LIFT_UP));
    EXPECT_TRUE(hydraulic_service_->execute_lift_action(common::LiftAction::LIFT_DOWN));
    EXPECT_TRUE(hydraulic_service_->execute_lift_action(common::LiftAction::STOP));
}

TEST_F(HydraulicServiceTest, SetMultiValveFlow) {
    hydraulic_service_->initialize();
    hydraulic_service_->enable_hydraulic_control(true);
    
    for (uint8_t valve_id = 0; valve_id < 4; ++valve_id) {
        EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(valve_id, 50));
        EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(valve_id, -50));
        EXPECT_TRUE(hydraulic_service_->set_multi_valve_flow(valve_id, 0));
    }
}

} // namespace core
} // namespace vcu
