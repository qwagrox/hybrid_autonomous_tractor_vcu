#include <gtest/gtest.h>
#include "vcu/can/can_interface.h"
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>

namespace vcu {
namespace can {
namespace test {

/**
 * @class MockCanInterface
 * @brief Mock implementation of ICanInterface for testing.
 */
class MockCanInterface : public ICanInterface {
public:
    MockCanInterface() : initialized_(false), receiving_(false), bitrate_(0) {}

    CanResult initialize(const std::string& interface_name, uint32_t bitrate) override {
        if (interface_name.empty()) {
            return CanResult::ERROR_INVALID_PARAM;
        }
        interface_name_ = interface_name;
        bitrate_ = bitrate;
        initialized_ = true;
        return CanResult::SUCCESS;
    }

    CanResult shutdown() override {
        stop_receive();
        initialized_ = false;
        return CanResult::SUCCESS;
    }

    CanResult send_frame(const CanFrame& frame) override {
        if (!initialized_) {
            return CanResult::ERROR_INIT;
        }
        sent_frames_.push_back(frame);
        
        // Simulate loopback for testing
        if (receive_callback_ && receiving_) {
            receive_callback_(frame);
        }
        
        return CanResult::SUCCESS;
    }

    void set_receive_callback(CanReceiveCallback callback) override {
        receive_callback_ = std::move(callback);
    }

    CanResult start_receive() override {
        if (!initialized_) {
            return CanResult::ERROR_INIT;
        }
        receiving_ = true;
        return CanResult::SUCCESS;
    }

    CanResult stop_receive() override {
        receiving_ = false;
        return CanResult::SUCCESS;
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
    const std::vector<CanFrame>& get_sent_frames() const {
        return sent_frames_;
    }

    void clear_sent_frames() {
        sent_frames_.clear();
    }

    void simulate_received_frame(const CanFrame& frame) {
        if (receive_callback_ && receiving_) {
            receive_callback_(frame);
        }
    }

private:
    bool initialized_;
    bool receiving_;
    std::string interface_name_;
    uint32_t bitrate_;
    CanReceiveCallback receive_callback_;
    std::vector<CanFrame> sent_frames_;
};

class CanInterfaceTest : public ::testing::Test {
protected:
    void SetUp() override {
        can_interface_ = std::make_unique<MockCanInterface>();
    }

    void TearDown() override {
        can_interface_.reset();
    }

    std::unique_ptr<MockCanInterface> can_interface_;
};

TEST_F(CanInterfaceTest, InitialState) {
    EXPECT_FALSE(can_interface_->is_ready());
    EXPECT_EQ(can_interface_->get_interface_name(), "");
    EXPECT_EQ(can_interface_->get_bitrate(), 0u);
}

TEST_F(CanInterfaceTest, SuccessfulInitialization) {
    CanResult result = can_interface_->initialize("vcan0", 500000);
    
    EXPECT_EQ(result, CanResult::SUCCESS);
    EXPECT_TRUE(can_interface_->is_ready());
    EXPECT_EQ(can_interface_->get_interface_name(), "vcan0");
    EXPECT_EQ(can_interface_->get_bitrate(), 500000u);
}

TEST_F(CanInterfaceTest, InvalidInitialization) {
    CanResult result = can_interface_->initialize("", 500000);
    
    EXPECT_EQ(result, CanResult::ERROR_INVALID_PARAM);
    EXPECT_FALSE(can_interface_->is_ready());
}

TEST_F(CanInterfaceTest, SendFrameWithoutInitialization) {
    CanFrame frame;
    frame.id = 0x123;
    frame.dlc = 8;
    
    CanResult result = can_interface_->send_frame(frame);
    
    EXPECT_EQ(result, CanResult::ERROR_INIT);
}

TEST_F(CanInterfaceTest, SendFrameAfterInitialization) {
    can_interface_->initialize("vcan0", 500000);
    
    CanFrame frame;
    frame.id = 0x123;
    frame.dlc = 4;
    frame.data = {0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00};
    
    CanResult result = can_interface_->send_frame(frame);
    
    EXPECT_EQ(result, CanResult::SUCCESS);
    
    const auto& sent_frames = can_interface_->get_sent_frames();
    ASSERT_EQ(sent_frames.size(), 1u);
    EXPECT_EQ(sent_frames[0].id, 0x123u);
    EXPECT_EQ(sent_frames[0].dlc, 4u);
    EXPECT_EQ(sent_frames[0].data[0], 0x01u);
    EXPECT_EQ(sent_frames[0].data[3], 0x04u);
}

TEST_F(CanInterfaceTest, ReceiveCallback) {
    can_interface_->initialize("vcan0", 500000);
    
    std::atomic<int> callback_count{0};
    CanFrame received_frame;
    
    can_interface_->set_receive_callback([&](const CanFrame& frame) {
        received_frame = frame;
        callback_count++;
    });
    
    can_interface_->start_receive();
    
    CanFrame test_frame;
    test_frame.id = 0x456;
    test_frame.dlc = 2;
    test_frame.data = {0xAA, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    can_interface_->simulate_received_frame(test_frame);
    
    // Give some time for callback processing
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    EXPECT_EQ(callback_count.load(), 1);
    EXPECT_EQ(received_frame.id, 0x456u);
    EXPECT_EQ(received_frame.dlc, 2u);
    EXPECT_EQ(received_frame.data[0], 0xAAu);
    EXPECT_EQ(received_frame.data[1], 0xBBu);
}

TEST_F(CanInterfaceTest, ReceiveWithoutCallback) {
    can_interface_->initialize("vcan0", 500000);
    can_interface_->start_receive();
    
    CanFrame test_frame;
    test_frame.id = 0x789;
    
    // Should not crash when no callback is set
    can_interface_->simulate_received_frame(test_frame);
}

TEST_F(CanInterfaceTest, StartReceiveWithoutInitialization) {
    CanResult result = can_interface_->start_receive();
    EXPECT_EQ(result, CanResult::ERROR_INIT);
}

TEST_F(CanInterfaceTest, StopReceive) {
    can_interface_->initialize("vcan0", 500000);
    
    std::atomic<int> callback_count{0};
    can_interface_->set_receive_callback([&](const CanFrame& frame) {
        callback_count++;
    });
    
    can_interface_->start_receive();
    can_interface_->stop_receive();
    
    CanFrame test_frame;
    test_frame.id = 0x999;
    can_interface_->simulate_received_frame(test_frame);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    EXPECT_EQ(callback_count.load(), 0); // Should not receive after stop
}

TEST_F(CanInterfaceTest, Shutdown) {
    can_interface_->initialize("vcan0", 500000);
    can_interface_->start_receive();
    
    EXPECT_TRUE(can_interface_->is_ready());
    
    CanResult result = can_interface_->shutdown();
    
    EXPECT_EQ(result, CanResult::SUCCESS);
    EXPECT_FALSE(can_interface_->is_ready());
}

TEST_F(CanInterfaceTest, MultipleSendFrames) {
    can_interface_->initialize("vcan0", 500000);
    
    for (uint32_t i = 0; i < 5; ++i) {
        CanFrame frame;
        frame.id = 0x100 + i;
        frame.dlc = 1;
        frame.data[0] = static_cast<uint8_t>(i);
        
        CanResult result = can_interface_->send_frame(frame);
        EXPECT_EQ(result, CanResult::SUCCESS);
    }
    
    const auto& sent_frames = can_interface_->get_sent_frames();
    ASSERT_EQ(sent_frames.size(), 5u);
    
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(sent_frames[i].id, 0x100u + i);
        EXPECT_EQ(sent_frames[i].data[0], static_cast<uint8_t>(i));
    }
}

TEST_F(CanInterfaceTest, FactoryFunction) {
    auto interface = create_can_interface();
    ASSERT_NE(interface, nullptr);
    EXPECT_FALSE(interface->is_ready());
}

} // namespace test
} // namespace can
} // namespace vcu
