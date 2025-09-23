#include "vcu/can/can_interface.h"
#include "vcu/can/socketcan_interface.h"

namespace vcu {
namespace can {

std::unique_ptr<ICanInterface> create_can_interface() {
    // For now, we only support SocketCAN on Linux
    // In the future, this could be extended to support other CAN interfaces
    // based on compile-time or runtime configuration
    return std::make_unique<SocketCanInterface>();
}

} // namespace can
} // namespace vcu
