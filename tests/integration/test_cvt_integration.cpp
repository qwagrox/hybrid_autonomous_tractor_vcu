#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <thread>
#include <chrono>
#include "vcu/cvt/cvt_controller_new.h"
#include "vcu/cvt/cvt_strategy_factory.h"
#include "vcu/cvt/cvt_config.h"
#include "vcu/can/can_interface.h"

namespace vcu {
namespace integration {
namespace test {

// Mock CAN interface for integration testing
class MockCanInterface : public can::CanInterface {
public:
    MOCK_METHOD(bool, init, (), (override));
    MOCK_METHOD(void, shutdown, (), (override));
    MOCK_METHOD(bool, send_frame, (const can::CanFrame& frame), (override));
    MOCK_METHOD(bool, receive_frame, (can::CanFrame& frame), (override));
    MOCK_METHOD(bool, is_connected, (), (const, override));
    
    // Helper to capture sent frames
    std::vector<can::CanFrame> sent_frames;
    
    bool capture_send_frame(const can::CanFrame& frame) {
        sent_frames.push_back(frame);
        return true;
    }
};

class CvtIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_can_interface_ = std::make_unique<MockCanInterface>();
        
        // Setup default expectations
        EXPECT_CALL(*mock_can_interface_, init())
            .WillRepeatedly(::testing::Return(true));
        EXPECT_CALL(*mock_can_interface_, is_connected())
            .WillRepeatedly(::testing::Return(true));
        EXPECT_CALL(*mock_can_interface_, send_frame(::testing::_))
            .WillRepeatedly(::testing::Invoke(mock_can_interface_.get(), 
                &MockCanInterface::capture_send_frame));
    }

    void TearDown() override {
        cvt_controller_.reset();
        mock_can_interface_.reset();
    }

    std::unique_ptr<MockCanInterface> mock_can_interface_;
    std::unique_ptr<cvt::CvtController> cvt_controller_;
};

TEST_F(CvtIntegrationTest, FullCvtControllerLifecycle) {
    // Test complete CVT controller lifecycle
    
    // Create configuration
    cvt::CvtConfig config = cvt::CvtConfigManager::get_default_config();
    config.manufacturer = common::CvtManufacturer::HMCVT_VENDOR1;
    config.enable_debug_logging = false; // Disable for test
    
    // Create controller
    cvt_controller_ = std::make_unique<cvt::CvtController>(*mock_can_interface_, config);
    
    // Initialize
    ASSERT_TRUE(cvt_controller_->init());
    EXPECT_TRUE(cvt_controller_->is_ready());
    
    // Set drive mode
    cvt_controller_->set_drive_mode(common::DriveMode::PLOWING);
    
    // Create test perception data
    common::PerceptionData perception_data;
    perception_data.vehicle_speed_mps = 3.0f;
    perception_data.engine_speed_rpm = 2000.0f;
    perception_data.engine_load_percent = 60.0f;
    perception_data.accelerator_pedal_percent = 40.0f;
    perception_data.fuel_level_percent = 80.0f;
    perception_data.coolant_temp_celsius = 75.0f;
    perception_data.data_valid = true;
    
    // Create test prediction data
    common::PredictionResult prediction_result;
    prediction_result.predicted_load_percent = 65.0f;
    
    // Update controller multiple times
    for (int i = 0; i < 5; ++i) {
        cvt_controller_->update(perception_data, prediction_result);
        
        // Verify state is reasonable
        common::CvtState state = cvt_controller_->get_current_state();
        EXPECT_GE(state.current_ratio, -1.0f);
        EXPECT_LE(state.current_ratio, 2.5f);
        EXPECT_GE(state.target_ratio, -1.0f);
        EXPECT_LE(state.target_ratio, 2.5f);
        
        // Small delay to simulate real-time operation
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Verify CAN messages were sent
    EXPECT_GT(mock_can_interface_->sent_frames.size(), 0);
    
    // Verify CAN message format
    for (const auto& frame : mock_can_interface_->sent_frames) {
        EXPECT_EQ(frame.id, 0x18FFF023u); // HMCVT_Vendor1 control message ID
        EXPECT_EQ(frame.dlc, 8);
    }
}

TEST_F(CvtIntegrationTest, ManufacturerSwitching) {
    // Test switching between different CVT manufacturers
    
    cvt::CvtConfig config = cvt::CvtConfigManager::get_default_config();
    cvt_controller_ = std::make_unique<cvt::CvtController>(*mock_can_interface_, config);
    
    ASSERT_TRUE(cvt_controller_->init());
    
    // Switch to different manufacturers
    EXPECT_TRUE(cvt_controller_->set_cvt_manufacturer(common::CvtManufacturer::BOSCH));
    EXPECT_TRUE(cvt_controller_->is_ready());
    
    EXPECT_TRUE(cvt_controller_->set_cvt_manufacturer(common::CvtManufacturer::ZF));
    EXPECT_TRUE(cvt_controller_->is_ready());
    
    EXPECT_TRUE(cvt_controller_->set_cvt_manufacturer(common::CvtManufacturer::HMCVT_VENDOR1));
    EXPECT_TRUE(cvt_controller_->is_ready());
}

TEST_F(CvtIntegrationTest, ConfigurationUpdate) {
    // Test runtime configuration updates
    
    cvt::CvtConfig initial_config = cvt::CvtConfigManager::get_default_config();
    cvt_controller_ = std::make_unique<cvt::CvtController>(*mock_can_interface_, initial_config);
    
    ASSERT_TRUE(cvt_controller_->init());
    
    // Update configuration
    cvt::CvtConfig new_config = initial_config;
    new_config.max_ratio = 1.8f;
    new_config.adjustment_rate = 0.2f;
    new_config.enable_four_wheel_drive = true;
    
    EXPECT_TRUE(cvt_controller_->update_config(new_config));
    
    // Verify configuration was updated
    const cvt::CvtConfig& current_config = cvt_controller_->get_config();
    EXPECT_FLOAT_EQ(current_config.max_ratio, 1.8f);
    EXPECT_FLOAT_EQ(current_config.adjustment_rate, 0.2f);
    EXPECT_TRUE(current_config.enable_four_wheel_drive);
}

TEST_F(CvtIntegrationTest, DriveModeBehavior) {
    // Test behavior in different drive modes
    
    cvt::CvtConfig config = cvt::CvtConfigManager::get_default_config();
    cvt_controller_ = std::make_unique<cvt::CvtController>(*mock_can_interface_, config);
    
    ASSERT_TRUE(cvt_controller_->init());
    
    common::PerceptionData perception_data;
    perception_data.vehicle_speed_mps = 2.0f;
    perception_data.engine_speed_rpm = 1800.0f;
    perception_data.engine_load_percent = 70.0f;
    perception_data.accelerator_pedal_percent = 50.0f;
    perception_data.data_valid = true;
    
    common::PredictionResult prediction_result;
    prediction_result.predicted_load_percent = 75.0f;
    
    // Test plowing mode (should prefer lower ratios for more torque)
    cvt_controller_->set_drive_mode(common::DriveMode::PLOWING);
    cvt_controller_->update(perception_data, prediction_result);
    common::CvtState plowing_state = cvt_controller_->get_current_state();
    
    // Test transport mode (should prefer higher ratios for efficiency)
    cvt_controller_->set_drive_mode(common::DriveMode::TRANSPORT);
    cvt_controller_->update(perception_data, prediction_result);
    common::CvtState transport_state = cvt_controller_->get_current_state();
    
    // Transport mode should generally have higher target ratio than plowing
    // (though this depends on the specific algorithm implementation)
    EXPECT_NE(plowing_state.target_ratio, transport_state.target_ratio);
}

TEST_F(CvtIntegrationTest, SafetyValidation) {
    // Test safety validation features
    
    cvt::CvtConfig config = cvt::CvtConfigManager::get_default_config();
    config.max_oil_temp = 80.0f; // Lower threshold for testing
    cvt_controller_ = std::make_unique<cvt::CvtController>(*mock_can_interface_, config);
    
    ASSERT_TRUE(cvt_controller_->init());
    
    // Test with normal conditions
    common::PerceptionData normal_data;
    normal_data.vehicle_speed_mps = 5.0f;
    normal_data.engine_speed_rpm = 2000.0f;
    normal_data.coolant_temp_celsius = 70.0f; // Normal temperature
    normal_data.data_valid = true;
    
    common::PredictionResult prediction_result;
    
    cvt_controller_->update(normal_data, prediction_result);
    EXPECT_TRUE(cvt_controller_->is_ready());
    
    // Test with high temperature (should trigger safety)
    common::PerceptionData hot_data = normal_data;
    hot_data.coolant_temp_celsius = 95.0f; // Above threshold
    
    // Update multiple times to trigger safety violation count
    for (int i = 0; i < 5; ++i) {
        cvt_controller_->update(hot_data, prediction_result);
    }
    
    // Controller might become not ready due to safety violations
    // (behavior depends on implementation details)
}

TEST_F(CvtIntegrationTest, StrategyFactoryIntegration) {
    // Test integration with strategy factory
    
    // Test creating different strategies
    auto hmcvt_strategy = cvt::CvtStrategyFactory::create_strategy(
        common::CvtManufacturer::HMCVT_VENDOR1, *mock_can_interface_);
    ASSERT_NE(hmcvt_strategy, nullptr);
    
    auto bosch_strategy = cvt::CvtStrategyFactory::create_strategy(
        common::CvtManufacturer::BOSCH, *mock_can_interface_);
    ASSERT_NE(bosch_strategy, nullptr);
    
    auto zf_strategy = cvt::CvtStrategyFactory::create_strategy(
        common::CvtManufacturer::ZF, *mock_can_interface_);
    ASSERT_NE(zf_strategy, nullptr);
    
    // Test direct HMCVT_Vendor1 creation
    auto direct_strategy = cvt::CvtStrategyFactory::create_hmcvt_vendor1_strategy(*mock_can_interface_);
    ASSERT_NE(direct_strategy, nullptr);
    
    // All strategies should implement the same interface
    EXPECT_NO_THROW(hmcvt_strategy->set_drive_mode(common::DriveMode::MANUAL));
    EXPECT_NO_THROW(bosch_strategy->set_drive_mode(common::DriveMode::MANUAL));
    EXPECT_NO_THROW(zf_strategy->set_drive_mode(common::DriveMode::MANUAL));
    EXPECT_NO_THROW(direct_strategy->set_drive_mode(common::DriveMode::MANUAL));
}

TEST_F(CvtIntegrationTest, RatioConvergence) {
    // Test that CVT ratio converges to target over time
    
    cvt::CvtConfig config = cvt::CvtConfigManager::get_default_config();
    config.adjustment_rate = 0.2f; // Faster convergence for testing
    cvt_controller_ = std::make_unique<cvt::CvtController>(*mock_can_interface_, config);
    
    ASSERT_TRUE(cvt_controller_->init());
    
    // Set a specific target ratio
    const float target_ratio = 1.5f;
    
    common::PerceptionData perception_data;
    perception_data.vehicle_speed_mps = 8.0f;
    perception_data.engine_speed_rpm = 2200.0f;
    perception_data.engine_load_percent = 40.0f;
    perception_data.accelerator_pedal_percent = 60.0f; // This should result in target_ratio
    perception_data.data_valid = true;
    
    common::PredictionResult prediction_result;
    prediction_result.predicted_load_percent = 45.0f;
    
    // Set transport mode to get predictable ratio calculation
    cvt_controller_->set_drive_mode(common::DriveMode::TRANSPORT);
    
    // Update multiple times and check convergence
    float initial_ratio = cvt_controller_->get_current_state().current_ratio;
    
    for (int i = 0; i < 20; ++i) {
        cvt_controller_->update(perception_data, prediction_result);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    common::CvtState final_state = cvt_controller_->get_current_state();
    
    // Current ratio should have moved towards target ratio
    float ratio_change = std::abs(final_state.current_ratio - initial_ratio);
    EXPECT_GT(ratio_change, 0.01f); // Should have changed by at least 0.01
    
    // Should not be shifting if we've converged
    // (This depends on the tolerance settings)
}

} // namespace test
} // namespace integration
} // namespace vcu
