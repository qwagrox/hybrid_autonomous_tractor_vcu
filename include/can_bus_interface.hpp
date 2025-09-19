// include/can_bus_interface.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <functional>
#include <string>
#include <vector>
#include <map>

namespace VCUCore {

class CANBusInterface {
public:
    CANBusInterface(const std::string& interface = "can0");
    ~CANBusInterface();

    bool initialize();
    bool sendCANFrame(const can_frame& frame);
    std::vector<can_frame> receiveCANFrames(int timeout_ms = 10);

    EngineData parseEngineData(const can_frame& frame);
    bool requestEngineData(uint32_t pgn);
    void requestCriticalEngineParameters();

    void registerMessageHandler(uint32_t can_id, std::function<void(const can_frame&)> handler);
    void processReceivedFrames();

    bool isInitialized() const { return isInitialized_; }
    std::string getInterfaceName() const { return interfaceName_; }

    bool checkBusStatus();
    bool resetBus();

private:
    int canSocket_;
    std::string interfaceName_;
    bool isInitialized_;
    std::map<uint32_t, std::function<void(const can_frame&)>> messageHandlers_;

    // J1939 PGNs
    static constexpr uint32_t PGN_ENGINE_TORQUE = 0x0CF00400;
    static constexpr uint32_t PGN_ENGINE_SPEED = 0x0CF00401;
    static constexpr uint32_t PGN_FUEL_CONSUMPTION = 0x0CF00402;
    static constexpr uint32_t PGN_ENGINE_LOAD = 0x0CF00403;

    // Manufacturer IDs
    static constexpr uint32_t JD_BASE_ID = 0x18FF1000;
    static constexpr uint32_t CASE_BASE_ID = 0x18CF2000;

    bool setupCANSocket();
    bool bindToInterface();
    bool setCANFilter();
    uint32_t calculateJ1939ID(uint8_t priority, uint8_t pdu_format, uint8_t pdu_specific, uint8_t source_addr);
};

} // namespace VCUCore

