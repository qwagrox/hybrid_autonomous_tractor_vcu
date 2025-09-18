// virtual_can_bus.hpp
#ifndef VIRTUAL_CAN_BUS_HPP
#define VIRTUAL_CAN_BUS_HPP

#include <vector>
#include <cstdint>
#include <queue>

struct CANMessage {
    uint32_t id;
    std::vector<uint8_t> data;
};

class VirtualCANBus {
public:
    void sendMessage(const CANMessage& msg) {
        message_queue_.push(msg);
    }

    bool receiveMessage(CANMessage& msg) {
        if (message_queue_.empty()) {
            return false;
        }
        msg = message_queue_.front();
        message_queue_.pop();
        return true;
    }

private:
    std::queue<CANMessage> message_queue_;
};

#endif // VIRTUAL_CAN_BUS_HPP

