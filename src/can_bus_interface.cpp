// src/can_bus_interface.cpp
#include "can_bus_interface.hpp"
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <system_error>

namespace VCUCore {

CANBusInterface::CANBusInterface(const std::string& interface) 
    : interfaceName_(interface), canSocket_(-1), isInitialized_(false) {}

CANBusInterface::~CANBusInterface() {
    if (canSocket_ >= 0) {
        close(canSocket_);
    }
}

bool CANBusInterface::initialize() {
    if (isInitialized_) {
        return true;
    }

    try {
        if (!setupCANSocket()) {
            throw std::runtime_error("Failed to setup CAN socket");
        }

        if (!bindToInterface()) {
            throw std::runtime_error("Failed to bind to CAN interface");
        }

        if (!setCANFilter()) {
            throw std::runtime_error("Failed to set CAN filter");
        }

        isInitialized_ = true;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "CAN initialization failed: " << e.what() << std::endl;
        return false;
    }
}

bool CANBusInterface::setupCANSocket() {
    canSocket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (canSocket_ < 0) {
        std::cerr << "Error creating CAN socket: " << strerror(errno) << std::endl;
        return false;
    }

    // 设置非阻塞模式
    int flags = fcntl(canSocket_, F_GETFL, 0);
    if (flags == -1) {
        close(canSocket_);
        return false;
    }
    if (fcntl(canSocket_, F_SETFL, flags | O_NONBLOCK) == -1) {
        close(canSocket_);
        return false;
    }

    return true;
}

bool CANBusInterface::bindToInterface() {
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interfaceName_.c_str(), IFNAMSIZ - 1);
    
    if (ioctl(canSocket_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error getting interface index: " << strerror(errno) << std::endl;
        close(canSocket_);
        return false;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(canSocket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error binding to CAN interface: " << strerror(errno) << std::endl;
        close(canSocket_);
        return false;
    }

    return true;
}

bool CANBusInterface::sendCANFrame(const can_frame& frame) {
    if (!isInitialized_) {
        std::cerr << "CAN bus not initialized" << std::endl;
        return false;
    }

    ssize_t bytes_sent = write(canSocket_, &frame, sizeof(frame));
    if (bytes_sent != sizeof(frame)) {
        std::cerr << "Error sending CAN frame: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

std::vector<can_frame> CANBusInterface::receiveCANFrames(int timeout_ms) {
    std::vector<can_frame> frames;
    
    if (!isInitialized_) {
        return frames;
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(canSocket_, &readfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(canSocket_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret > 0 && FD_ISSET(canSocket_, &readfds)) {
        can_frame frame;
        while (read(canSocket_, &frame, sizeof(frame)) > 0) {
            frames.push_back(frame);
        }
    }

    return frames;
}

EngineData CANBusInterface::parseEngineData(const can_frame& frame) {
    EngineData data;
    uint32_t pgn = frame.can_id & 0x1FFFF00;

    switch (pgn) {
        case PGN_ENGINE_TORQUE:
            data.actualTorque = (frame.data[0] * 1.0f) - 125.0f;
            data.percentLoad = frame.data[1] * 0.4f;
            break;
            
        case PGN_ENGINE_SPEED: {
            uint16_t speed = (frame.data[0] << 8) | frame.data[1];
            data.speed = speed * 0.125f;
            break;
        }
            
        case PGN_FUEL_CONSUMPTION: {
            uint16_t fuel = (frame.data[0] << 8) | frame.data[1];
            data.fuelRate = fuel * 0.05f;
            break;
        }
            
        case PGN_ENGINE_LOAD:
            data.percentLoad = frame.data[0] * 1.0f;
            data.boostPressure = frame.data[1] * 2.0f;
            break;
            
        default:
            break;
    }

    data.timestamp = std::chrono::duration_cast<Timestamp>(
        std::chrono::system_clock::now().time_since_epoch());
        
    return data;
}

bool CANBusInterface::requestEngineData(uint32_t pgn) {
    can_frame requestFrame;
    requestFrame.can_id = 0x18EA00F9;
    requestFrame.can_dlc = 8;

    std::memset(requestFrame.data, 0, 8);
    requestFrame.data[0] = pgn & 0xFF;
    requestFrame.data[1] = (pgn >> 8) & 0xFF;
    requestFrame.data[2] = (pgn >> 16) & 0xFF;
    requestFrame.data[3] = 0x00;
    requestFrame.data[4] = 0xF9;
    requestFrame.data[5] = 0xFF;
    requestFrame.data[6] = 0xFF;
    requestFrame.data[7] = 0xFF;

    return sendCANFrame(requestFrame);
}

void CANBusInterface::requestCriticalEngineParameters() {
    uint32_t pgns[] = {
        PGN_ENGINE_TORQUE,
        PGN_ENGINE_SPEED,
        PGN_ENGINE_LOAD,
        PGN_FUEL_CONSUMPTION
    };

    for (auto pgn : pgns) {
        requestEngineData(pgn);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void CANBusInterface::registerMessageHandler(uint32_t can_id, std::function<void(const can_frame&)> handler) {
    messageHandlers_[can_id] = handler;
}

void CANBusInterface::processReceivedFrames() {
    auto frames = receiveCANFrames(5);
    for (const auto& frame : frames) {
        if (messageHandlers_.find(frame.can_id) != messageHandlers_.end()) {
            messageHandlers_[frame.can_id](frame);
        }
    }
}

bool CANBusInterface::checkBusStatus() {
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interfaceName_.c_str(), IFNAMSIZ - 1);
    
    if (ioctl(canSocket_, SIOCGIFFLAGS, &ifr) < 0) {
        return false;
    }

    return (ifr.ifr_flags & IFF_UP) && (ifr.ifr_flags & IFF_RUNNING);
}

bool CANBusInterface::resetBus() {
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interfaceName_.c_str(), IFNAMSIZ - 1);
    
    // 先关闭接口
    ifr.ifr_flags = 0;
    if (ioctl(canSocket_, SIOCSIFFLAGS, &ifr) < 0) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 重新开启接口
    ifr.ifr_flags = IFF_UP | IFF_RUNNING;
    if (ioctl(canSocket_, SIOCSIFFLAGS, &ifr) < 0) {
        return false;
    }

    return true;
}

bool CANBusInterface::setCANFilter() {
    struct can_filter filter[4];
    
    // 设置J1939消息过滤器
    filter[0].can_id = 0x0CF00000;
    filter[0].can_mask = 0x1FFFF00;
    
    filter[1].can_id = 0x18EA0000;
    filter[1].can_mask = 0x1FFFF00;
    
    filter[2].can_id = JD_BASE_ID;
    filter[2].can_mask = 0x1FFF0000;
    
    filter[3].can_id = CASE_BASE_ID;
    filter[3].can_mask = 0x1FFF0000;

    if (setsockopt(canSocket_, SOL_CAN_RAW, CAN_RAW_FILTER, 
                  &filter, sizeof(filter)) < 0) {
        std::cerr << "Error setting CAN filter: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

uint32_t CANBusInterface::calculateJ1939ID(uint8_t priority, uint8_t pdu_format, 
                                         uint8_t pdu_specific, uint8_t source_addr) {
    return (priority << 26) | (pdu_format << 16) | (pdu_specific << 8) | source_addr;
}

} // namespace VCUCore