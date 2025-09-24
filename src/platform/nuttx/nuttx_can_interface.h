#ifndef NUTTX_CAN_INTERFACE_H
#define NUTTX_CAN_INTERFACE_H

#include "vcu/can/can_interface.h"
#include <nuttx/can/can.h>

class NuttxCanInterface : public CanInterface {
private:
    int can_fd_;
    bool initialized_;

public:
    NuttxCanInterface();
    ~NuttxCanInterface() override;
    
    CanResult initialize(const std::string& interface_name) override;
    CanResult shutdown() override;
    bool is_initialized() const override;
    
    CanResult send_frame(const CanFrame& frame) override;
    CanResult receive_frame(CanFrame& frame) override;
    
    CanResult set_receive_callback(std::function<void(const CanFrame&)> callback) override;
    CanResult start_receive() override;
    CanResult stop_receive() override;
};

#endif // NUTTX_CAN_INTERFACE_H
