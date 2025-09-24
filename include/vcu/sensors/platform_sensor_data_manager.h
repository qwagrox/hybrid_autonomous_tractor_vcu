#ifndef PLATFORM_SENSOR_DATA_MANAGER_H
#define PLATFORM_SENSOR_DATA_MANAGER_H

#include "vcu/common/vcu_data_types.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/platform/time_interface.h"
#include "vcu/platform/mutex_interface.h"
#include "vcu/can/can_interface.h"
#include "vcu/can/can_frame.h"
#include <memory>
#include <functional>

namespace vcu {
namespace sensors {

enum class SensorDataResult {
    SUCCESS = 0,
    ERROR_INVALID_PARAMETER,
    ERROR_CAN_COMM,
    ERROR_TIMEOUT,
    ERROR_NOT_INITIALIZED
};

class PlatformSensorDataManager {
public:
    explicit PlatformSensorDataManager(
        PlatformInterface* platform,
        std::shared_ptr<can::ICanInterface> can_interface
    );
    ~PlatformSensorDataManager();

    // 初始化和关闭
    SensorDataResult initialize(uint32_t data_timeout_ms = 1000);
    SensorDataResult shutdown();

    // 数据获取
    SensorDataResult get_current_data(common::PerceptionData& data);
    bool is_data_fresh() const;
    uint32_t get_data_age_ms() const;

    // 回调设置
    void set_data_update_callback(std::function<void(const common::PerceptionData&)> callback);

private:
    // 平台接口
    PlatformInterface* platform_;
    std::unique_ptr<TimeInterface> time_interface_;
    std::unique_ptr<MutexInterface> data_mutex_;
    
    // CAN接口
    std::shared_ptr<can::ICanInterface> can_interface_;
    
    // 数据存储
    common::PerceptionData perception_data_;
    uint64_t last_update_time_ms_;
    uint32_t data_timeout_ms_;
    bool is_initialized_;
    
    // 回调函数
    std::function<void(const common::PerceptionData&)> data_callback_;
    
    // 内部方法
    void on_can_frame_received(const can::CanFrame& frame);
    void update_perception_data();
    void notify_data_callback();
    
    // 数据解析方法
    void parse_engine_data(const can::CanFrame& frame);
    void parse_vehicle_data(const can::CanFrame& frame);
    void parse_cvt_status(const can::CanFrame& frame);
};

} // namespace sensors
} // namespace vcu

#endif // PLATFORM_SENSOR_DATA_MANAGER_H
