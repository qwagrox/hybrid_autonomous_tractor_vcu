#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "vcu/cvt/cvt_strategy_factory.h"
#include "vcu/cvt/cvt_strategy.h"
#include "vcu/can/can_interface.h"

namespace vcu {
namespace cvt {
namespace test {

// Mock CAN interface for testing
class MockCanInterface : public can::CanInterface {
public:
    MOCK_METHOD(bool, init, (), (override));
    MOCK_METHOD(void, shutdown, (), (override));
    MOCK_METHOD(bool, send_frame, (const can::CanFrame& frame), (override));
    MOCK_METHOD(bool, receive_frame, (can::CanFrame& frame), (override));
    MOCK_METHOD(bool, is_connected, (), (const, override));
};

class CvtStrategyFactoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_can_interface_ = std::make_unique<MockCanInterface>();
    }

    void TearDown() override {
        mock_can_interface_.reset();
    }

    std::unique_ptr<MockCanInterface> mock_can_interface_;
};

TEST_F(CvtStrategyFactoryTest, CreateHMCVTVendor1Strategy) {
    // Test creating HMCVT_Vendor1 strategy
    auto strategy = CvtStrategyFactory::create_strategy(
        common::CvtManufacturer::HMCVT_VENDOR1, 
        *mock_can_interface_
    );

    ASSERT_NE(strategy, nullptr);
    
    // Verify the strategy can be initialized
    EXPECT_CALL(*mock_can_interface_, init())
        .WillOnce(::testing::Return(true));
    
    // Note: We can't directly test init() without modifying the strategy
    // to accept mock interfaces, but we can verify the strategy exists
}

TEST_F(CvtStrategyFactoryTest, CreateStrategyForUnknownManufacturer) {
    // Test creating strategy for unknown manufacturer (should default to HMCVT_Vendor1)
    auto strategy = CvtStrategyFactory::create_strategy(
        common::CvtManufacturer::UNKNOWN, 
        *mock_can_interface_
    );

    ASSERT_NE(strategy, nullptr);
}

TEST_F(CvtStrategyFactoryTest, CreateStrategyForBoschManufacturer) {
    // Test creating strategy for Bosch manufacturer (currently uses HMCVT_Vendor1)
    auto strategy = CvtStrategyFactory::create_strategy(
        common::CvtManufacturer::BOSCH, 
        *mock_can_interface_
    );

    ASSERT_NE(strategy, nullptr);
}

TEST_F(CvtStrategyFactoryTest, CreateStrategyForZFManufacturer) {
    // Test creating strategy for ZF manufacturer (currently uses HMCVT_Vendor1)
    auto strategy = CvtStrategyFactory::create_strategy(
        common::CvtManufacturer::ZF, 
        *mock_can_interface_
    );

    ASSERT_NE(strategy, nullptr);
}

TEST_F(CvtStrategyFactoryTest, CreateHMCVTVendor1StrategyDirect) {
    // Test direct creation of HMCVT_Vendor1 strategy
    auto strategy = CvtStrategyFactory::create_hmcvt_vendor1_strategy(*mock_can_interface_);

    ASSERT_NE(strategy, nullptr);
}

TEST_F(CvtStrategyFactoryTest, StrategyImplementsInterface) {
    // Test that created strategy implements the CvtStrategy interface
    auto strategy = CvtStrategyFactory::create_strategy(
        common::CvtManufacturer::HMCVT_VENDOR1, 
        *mock_can_interface_
    );

    ASSERT_NE(strategy, nullptr);
    
    // Test that we can call interface methods (they should not throw)
    EXPECT_NO_THROW(strategy->set_drive_mode(common::DriveMode::MANUAL));
    EXPECT_NO_THROW(strategy->set_target_ratio(1.0f));
    EXPECT_NO_THROW(strategy->get_current_state());
}

} // namespace test
} // namespace cvt
} // namespace vcu
