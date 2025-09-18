#ifndef ISOBUS_ADAPTER_HPP
#define ISOBUS_ADAPTER_HPP

#include <string>
#include <cstdint>
#include <functional>

/**
 * @class ISOBUSAdapter
 * @brief ISOBUS (ISO 11783) 协议适配器桩代码
 * 
 * 这是一个用于开发和测试的模拟实现。在实际部署中，它将被替换为
 * 与真实ISOBUS硬件和协议栈交互的完整实现。
 */
class ISOBUSAdapter {
public:
    ISOBUSAdapter() = default;
    ~ISOBUSAdapter() = default;

    /**
     * @brief 发送一个设置数值的通用ISOBUS指令
     * @param destination 目标设备地址
     * @param pgn 参数组编号 (Parameter Group Number)
     * @param value 要设置的数值
     * @return 如果指令成功发送，返回true
     */
    bool sendValueCommand(uint8_t destination, uint32_t pgn, double value) {
        // 在此模拟发送指令
        printf("[ISOBUS STUB] Sending PGN %u to %d with value %f\n", pgn, destination, value);
        return true;
    }

    /**
     * @brief 发送一个布尔开关指令
     * @param destination 目标设备地址
     * @param pgn 参数组编号
     * @param enable true为开启，false为关闭
     * @return 如果指令成功发送，返回true
     */
    bool sendEnableCommand(uint8_t destination, uint32_t pgn, bool enable) {
        // 在此模拟发送指令
        printf("[ISOBUS STUB] Sending PGN %u to %d with state %d\n", pgn, destination, enable);
        return true;
    }

    // 在实际应用中，这里还会有注册回调函数以接收来自农具的数据等功能
    // using DataCallback = std::function<void(uint32_t pgn, const uint8_t* data, uint8_t size)>;
    // void registerDataCallback(DataCallback callback);
};

#endif // ISOBUS_ADAPTER_HPP

