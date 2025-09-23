#include <gtest/gtest.h>
#include "vcu/sensors/sensor_data_manager.h"
#include "vcu/can/can_interface.h"
#include "vcu/can/j1939_protocol.h"
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstring>

namespace vcu {
namespace sensors {
namespace test {

/**
 * @class MockCanInterface
 * @brief Mock CAN interface for testing sensor data manager.
 */
class MockCanInterface : public can::ICanInterface {
public:
    MockCanInterface() : initialized_(false), receiving_(false), bitrate_(0) {}

    can::CanResult initialize(const std::string& interface_name, uint32_t bitrate) override {
        interface_name_ = interface_name;
        bitrate_ = bitrate;
        initialized_ = true;
        return can::CanResult::SUCCESS;
    }

    can::CanResult shutdown() override {
        stop_receive();
        initialized_ = false;
        return can::CanResult::SUCCESS;
    }

    can::CanResult send_frame(const can::CanFrame& frame) override {
        sent_frames_.push_back(frame);
        return can::CanResult::SUCCESS;
    }

    void set_receive_callback(can::CanReceiveCallback callback) override {
        receive_callback_ = std::move(callback);
    }

    can::CanResult start_receive() override {
        receiving_ = true;
        return can::CanResult::SUCCESS;
    }

    can::CanResult stop_receive() override {
        receiving_ = false;
        return can::CanResult::SUCCESS;
    }

    bool is_ready() const override {
        return initialized_;
    }

    std::string get_interface_name() const override {
        return interface_name_;
    }

    uint32_t get_bitrate() const override {
        return bitrate_;
    }

    // Test helper methods
    void simulate_received_frame(const can::CanFrame& frame) {
        if (receive_callback_ && receiving_) {
            receive_callback_(frame);
        }
    }

    void simulate_engine_data(float rpm, float load, float pedal) {
        can::CanFrame frame;
        frame.id = 0x0CF00400; // EEC1 message
        frame.is_extended = true;
        frame.dlc = 8;
        
        // Encode engine data
        uint16_t raw_speed = static_cast<uint16_t>(rpm / 0.125f);
        uint8_t raw_load = static_cast<uint8_t>(load / 0.5f);
        
        frame.data[2] = raw_load;
        frame.data[3] = static_cast<uint8_t>(raw_speed & 0xFF);
        frame.data[4] = static_cast<uint8_t>((raw_speed >> 8) & 0xFF);
        
        simulate_received_frame(frame);
    }

    void simulate_vehicle_speed(float speed_mps) {
        can::CanFrame frame;
        frame.id = 0x18FEF100; // CCVS1 message
        frame.is_extended = true;
        frame.dlc = 8;
        
        // Convert m/s to km/h and encode
        float speed_kmh = speed_mps * 3.6f;
        uint16_t raw_speed = static_cast<uint16_t>(speed_kmh * 256);
        
        frame.data[1] = static_cast<uint8_t>(raw_speed & 0xFF);
        frame.data[2] = static_cast<uint8_t>((raw_speed >> 8) & 0xFF);
        
        simulate_received_frame(frame);
    }

    void simulate_cvt_status(float ratio, bool shifting, uint8_t fault) {
        can::CanFrame frame;
        frame.id = 0x0CFF1100; // CVT status message
        frame.is_extended = true;
        frame.dlc = 8;
        
        // Encode CVT status
        std::memcpy(frame.data.data(), &ratio, sizeof(float));
        frame.data[4] = shifting ? 0x01 : 0x00;
        frame.data[5] = fault;
        
        simulate_received_frame(frame);
    }

private:
    bool initialized_;
    bool receiving_;
    std::string interface_name_;
    uint32_t bitrate_;
    can::CanReceiveCallback receive_callback_;
    std::vector<can::CanFrame> sent_frames_;
};

class SensorDataManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_can_ = std::make_shared<MockCanInterface>();
        mock_can_->initialize("vcan0", 500000);
        
        sensor_manager_ = std::make_unique<SensorDataManager>(mock_can_);
    }

    void TearDown() override {
        sensor_manager_.reset();
        mock_can_.reset();
    }

    std::shared_ptr<MockCanInterface> mock_can_;
    std::unique_ptr<SensorDataManager> sensor_manager_;
};

TEST_F(SensorDataManagerTest, InitialState) {
    EXPECT_FALSE(sensor_manager_->is_ready());
    
    common::PerceptionData data;
    auto result = sensor_manager_->get_current_data(data);
    EXPECT_EQ(result, SensorDataResult::ERROR_INIT);
}

TEST_F(SensorDataManagerTest, SuccessfulInitialization) {
    auto result = sensor_manager_->initialize(1000);
    
    EXPECT_EQ(result, SensorDataResult::SUCCESS);
    EXPECT_TRUE(sensor_manager_->is_ready());
}

TEST_F(SensorDataManagerTest, InitializationWithoutCAN) {
    auto sensor_manager_no_can = std::make_unique<SensorDataManager>(nullptr);
    
    auto result = sensor_manager_no_can->initialize();
    
    EXPECT_EQ(result, SensorDataResult::ERROR_CAN_COMM);
    EXPECT_FALSE(sensor_manager_no_can->is_ready());
}

TEST_F(SensorDataManagerTest, EngineDataReception) {
    sensor_manager_->initialize(1000);
    
    // Simulate engine data
    mock_can_->simulate_engine_data(1500.0f, 75.0f, 80.0f); // 1500 RPM, 75% load, 80% pedal
    
    // Give some time for processing
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    common::PerceptionData data;
    auto result = sensor_manager_->get_current_data(data);
    
    EXPECT_EQ(result, SensorDataResult::SUCCESS);
    EXPECT_TRUE(data.data_valid);
    EXPECT_FLOAT_EQ(data.engine_speed_rpm, 1500.0f);
    EXPECT_FLOAT_EQ(data.engine_load_percent, 75.0f);
    EXPECT_FLOAT_EQ(data.accelerator_pedal_percent, 80.0f);
}

TEST_F(SensorDataManagerTest, VehicleSpeedReception) {
    sensor_manager_->initialize(1000);
    
    // Simulate vehicle speed data
    mock_can_->simulate_vehicle_speed(15.0f); // 15 m/s
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    common::PerceptionData data;
    auto result = sensor_manager_->get_current_data(data);
    
    EXPECT_EQ(result, SensorDataResult::SUCCESS);
    EXPECT_TRUE(data.data_valid);
    EXPECT_NEAR(data.vehicle_speed_mps, 15.0f, 0.1f);
}

TEST_F(SensorDataManagerTest, CvtStatusReception) {
    sensor_manager_->initialize(1000);
    
    // Simulate CVT status data
    mock_can_->simulate_cvt_status(2.5f, true, 0); // Ratio 2.5, shifting, no fault
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    common::PerceptionData data;
    auto result = sensor_manager_->get_current_data(data);
    
    EXPECT_EQ(result, SensorDataResult::SUCCESS);
    EXPECT_TRUE(data.data_valid);
    EXPECT_FLOAT_EQ(data.current_transmission_ratio, 2.5f);
    EXPECT_TRUE(data.is_transmission_shifting);
}

TEST_F(SensorDataManagerTest, DataConsolidation) {
    sensor_manager_->initialize(1000);
    
    // Simulate multiple sensor inputs
    mock_can_->simulate_engine_data(2000.0f, 90.0f, 85.0f);
    mock_can_->simulate_vehicle_speed(8.0f);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    common::PerceptionData data;
    auto result = sensor_manager_->get_current_data(data);
    
    EXPECT_EQ(result, SensorDataResult::SUCCESS);
    EXPECT_TRUE(data.data_valid);
    
    // Check consolidated data
    EXPECT_FLOAT_EQ(data.engine_speed_rpm, 2000.0f);
    EXPECT_FLOAT_EQ(data.engine_load_percent, 90.0f);
    EXPECT_FLOAT_EQ(data.vehicle_speed_mps, 8.0f);
    
    // Check derived values
    EXPECT_GT(data.load_factor, 0.0f);
    EXPECT_LE(data.load_factor, 1.0f);
}

TEST_F(SensorDataManagerTest, TerrainTypeDetection) {
    sensor_manager_->initialize(1000);
    
    // Simulate rough terrain conditions (high load, low speed)
    mock_can_->simulate_engine_data(1800.0f, 95.0f, 90.0f);
    mock_can_->simulate_vehicle_speed(3.0f);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    common::PerceptionData data;
    sensor_manager_->get_current_data(data);
    
    EXPECT_EQ(data.terrain_type, common::TerrainType::ROUGH);
    
    // Simulate smooth terrain conditions (low load, higher speed)
    mock_can_->simulate_engine_data(1500.0f, 40.0f, 50.0f);
    mock_can_->simulate_vehicle_speed(12.0f);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    sensor_manager_->get_current_data(data);
    EXPECT_EQ(data.terrain_type, common::TerrainType::SMOOTH);
}

TEST_F(SensorDataManagerTest, DataCallback) {
    sensor_manager_->initialize(1000);
    
    std::atomic<int> callback_count{0};
    common::PerceptionData callback_data;
    
    sensor_manager_->set_data_callback([&](const common::PerceptionData& data) {
        callback_data = data;
        callback_count++;
    });
    
    // Simulate sensor data
    mock_can_->simulate_engine_data(1600.0f, 60.0f, 65.0f);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    EXPECT_GT(callback_count.load(), 0);
    EXPECT_TRUE(callback_data.data_valid);
    EXPECT_FLOAT_EQ(callback_data.engine_speed_rpm, 1600.0f);
}

TEST_F(SensorDataManagerTest, DataTimeout) {
    sensor_manager_->initialize(50); // 50ms timeout
    
    // Simulate initial data
    mock_can_->simulate_engine_data(1400.0f, 50.0f, 55.0f);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Data should be fresh
    EXPECT_TRUE(sensor_manager_->is_data_fresh());
    EXPECT_LT(sensor_manager_->get_data_age_ms(), 50u);
    
    common::PerceptionData data;
    auto result = sensor_manager_->get_current_data(data);
    EXPECT_EQ(result, SensorDataResult::SUCCESS);
    
    // Wait for timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    
    // Data should be stale
    EXPECT_FALSE(sensor_manager_->is_data_fresh());
    EXPECT_GT(sensor_manager_->get_data_age_ms(), 50u);
    
    result = sensor_manager_->get_current_data(data);
    EXPECT_EQ(result, SensorDataResult::ERROR_TIMEOUT);
}

TEST_F(SensorDataManagerTest, Shutdown) {
    sensor_manager_->initialize(1000);
    EXPECT_TRUE(sensor_manager_->is_ready());
    
    auto result = sensor_manager_->shutdown();
    
    EXPECT_EQ(result, SensorDataResult::SUCCESS);
    EXPECT_FALSE(sensor_manager_->is_ready());
}

TEST_F(SensorDataManagerTest, InvalidSensorData) {
    sensor_manager_->initialize(1000);
    
    // Simulate invalid CAN frame (unsupported message)
    can::CanFrame invalid_frame;
    invalid_frame.id = 0x12345678;
    invalid_frame.is_extended = true;
    invalid_frame.dlc = 8;
    
    mock_can_->simulate_received_frame(invalid_frame);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Should not have valid data
    common::PerceptionData data;
    auto result = sensor_manager_->get_current_data(data);
    EXPECT_NE(result, SensorDataResult::SUCCESS);
}

TEST_F(SensorDataManagerTest, MultipleDataUpdates) {
    sensor_manager_->initialize(1000);
    
    std::atomic<int> callback_count{0};
    sensor_manager_->set_data_callback([&](const common::PerceptionData& data) {
        callback_count++;
    });
    
    // Send multiple updates
    for (int i = 0; i < 5; ++i) {
        mock_can_->simulate_engine_data(1000.0f + i * 100, 50.0f + i * 5, 60.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    EXPECT_GE(callback_count.load(), 5);
    
    common::PerceptionData data;
    sensor_manager_->get_current_data(data);
    EXPECT_FLOAT_EQ(data.engine_speed_rpm, 1400.0f); // Last update
}

} // namespace test
} // namespace sensors
} // namespace vcu
