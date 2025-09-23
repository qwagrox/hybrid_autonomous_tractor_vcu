#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "vcu/sensors/sensor_data_manager.h"
#include "vcu/can/can_interface.h"
#include <memory>

namespace vcu {
namespace sensors {
namespace test {

class SimpleMockCanInterface : public can::ICanInterface {
public:
    SimpleMockCanInterface() : initialized_(false) {}

    can::CanResult initialize(const std::string&, uint32_t) override {
        initialized_ = true;
        return can::CanResult::SUCCESS;
    }

    can::CanResult shutdown() override {
        initialized_ = false;
        return can::CanResult::SUCCESS;
    }

    can::CanResult send_frame(const can::CanFrame&) override {
        return can::CanResult::SUCCESS;
    }

    void set_receive_callback(can::CanReceiveCallback callback) override {
        callback_ = std::move(callback);
    }

    can::CanResult start_receive() override {
        return can::CanResult::SUCCESS;
    }

    can::CanResult stop_receive() override {
        return can::CanResult::SUCCESS;
    }

    bool is_ready() const override {
        return initialized_;
    }

    std::string get_interface_name() const override {
        return "test";
    }

    uint32_t get_bitrate() const override {
        return 500000;
    }

private:
    bool initialized_;
    can::CanReceiveCallback callback_;
};

class SimpleSensorTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_can_ = std::make_shared<SimpleMockCanInterface>();
        mock_can_->initialize("test", 500000); // Initialize CAN interface first
        sensor_manager_ = std::make_unique<SensorDataManager>(mock_can_);
    }

    std::shared_ptr<SimpleMockCanInterface> mock_can_;
    std::unique_ptr<SensorDataManager> sensor_manager_;
};

TEST_F(SimpleSensorTest, BasicInitialization) {
    EXPECT_FALSE(sensor_manager_->is_ready());
    
    auto result = sensor_manager_->initialize(1000);
    EXPECT_EQ(result, SensorDataResult::SUCCESS);
    EXPECT_TRUE(sensor_manager_->is_ready());
}

TEST_F(SimpleSensorTest, DataTimeout) {
    // Test data age calculation directly
    sensor_manager_->initialize(1000); // 1 second timeout
    
    // Check that data age increases over time
    uint32_t age1 = sensor_manager_->get_data_age_ms();
    uint32_t age2 = sensor_manager_->get_data_age_ms();
    
    // Age should be non-negative and reasonable (less than 1 second in test)
    EXPECT_GE(age1, 0);
    EXPECT_GE(age2, age1);
    EXPECT_LT(age2, 1000); // Should be less than 1 second in a fast test
}

TEST_F(SimpleSensorTest, Shutdown) {
    sensor_manager_->initialize(1000);
    EXPECT_TRUE(sensor_manager_->is_ready());
    
    auto result = sensor_manager_->shutdown();
    EXPECT_EQ(result, SensorDataResult::SUCCESS);
    EXPECT_FALSE(sensor_manager_->is_ready());
}

} // namespace test
} // namespace sensors
} // namespace vcu
