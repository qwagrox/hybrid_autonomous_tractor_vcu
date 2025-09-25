#ifndef MOCK_CAN_INTERFACE_H
#define MOCK_CAN_INTERFACE_H

#include "vcu/can/can_interface.h"

namespace vcu {
namespace can {
namespace test {

class MockCanInterface : public ICanInterface {
public:
    CanResult initialize(const std::string& interface_name, uint32_t bitrate) override {
        interface_name_ = interface_name;
        bitrate_ = bitrate;
        is_ready_ = true;
        return CanResult::SUCCESS;
    }
    
    CanResult shutdown() override {
        is_ready_ = false;
        return CanResult::SUCCESS;
    }
    
    CanResult send_frame(const CanFrame& frame) override {
        last_sent_frame_ = frame;
        return CanResult::SUCCESS;
    }
    
    void set_receive_callback(CanReceiveCallback callback) override {
        receive_callback_ = callback;
    }
    
    CanResult start_receive() override {
        return CanResult::SUCCESS;
    }
    
    CanResult stop_receive() override {
        return CanResult::SUCCESS;
    }
    
    bool is_ready() const override {
        return is_ready_;
    }
    
    std::string get_interface_name() const override {
        return interface_name_;
    }
    
    uint32_t get_bitrate() const override {
        return bitrate_;
    }

    // Test helper methods
    const CanFrame& get_last_sent_frame() const { return last_sent_frame_; }
    void simulate_receive(const CanFrame& frame) {
        if (receive_callback_) {
            receive_callback_(frame);
        }
    }

private:
    std::string interface_name_ = "mock_can";
    uint32_t bitrate_ = 250000;
    bool is_ready_ = false;
    CanFrame last_sent_frame_;
    CanReceiveCallback receive_callback_;
};

} // namespace test
} // namespace can
} // namespace vcu

#endif // MOCK_CAN_INTERFACE_H
