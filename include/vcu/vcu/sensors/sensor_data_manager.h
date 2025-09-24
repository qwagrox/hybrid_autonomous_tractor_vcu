#ifndef SENSOR_DATA_MANAGER_H
#define SENSOR_DATA_MANAGER_H

#include "vcu/common/vcu_data_types.h"
#include "vcu/can/can_interface.h"
#include "vcu/can/j1939_protocol.h"
#include <memory>
#include <mutex>
#include <chrono>
#include <functional>

namespace vcu {
namespace sensors {

/**
 * @brief Callback function type for sensor data updates.
 * 
 * This callback is invoked whenever new sensor data is available.
 */
using SensorDataCallback = std::function<void(const common::PerceptionData& data)>;

/**
 * @enum SensorDataResult
 * @brief Result codes for sensor data operations.
 */
enum class SensorDataResult {
    SUCCESS = 0,        ///< Operation completed successfully
    ERROR_INIT,         ///< Failed to initialize sensor data manager
    ERROR_CAN_COMM,     ///< CAN communication error
    ERROR_TIMEOUT,      ///< Sensor data timeout
    ERROR_INVALID_DATA  ///< Invalid sensor data received
};

/**
 * @class SensorDataManager
 * @brief Manages sensor data collection from CAN bus and provides unified access.
 *
 * This class collects sensor data from various sources via CAN bus using J1939 protocol,
 * validates the data, and provides a unified interface for accessing current sensor readings.
 * It maintains data freshness and provides callbacks for real-time data updates.
 */
class SensorDataManager {
public:
    /**
     * @brief Constructs a new SensorDataManager.
     * @param can_interface Shared pointer to the CAN interface for communication.
     */
    explicit SensorDataManager(std::shared_ptr<can::ICanInterface> can_interface);

    /**
     * @brief Destroys the SensorDataManager and cleans up resources.
     */
    ~SensorDataManager();

    /**
     * @brief Initializes the sensor data manager.
     * 
     * @param data_timeout_ms Timeout in milliseconds for considering sensor data stale.
     * @return SensorDataResult::SUCCESS on success, error code otherwise.
     */
    SensorDataResult initialize(uint32_t data_timeout_ms = 1000);

    /**
     * @brief Shuts down the sensor data manager.
     * 
     * @return SensorDataResult::SUCCESS on success, error code otherwise.
     */
    SensorDataResult shutdown();

    /**
     * @brief Gets the current perception data.
     * 
     * @param data Output structure to store the current sensor data.
     * @return SensorDataResult::SUCCESS if data is valid and fresh, error code otherwise.
     */
    SensorDataResult get_current_data(common::PerceptionData& data) const;

    /**
     * @brief Sets a callback function for real-time sensor data updates.
     * 
     * @param callback The callback function to invoke when new data is available.
     */
    void set_data_callback(SensorDataCallback callback);

    /**
     * @brief Checks if the sensor data manager is initialized and ready.
     * 
     * @return True if ready for operation, false otherwise.
     */
    bool is_ready() const;

    /**
     * @brief Gets the age of the current sensor data in milliseconds.
     * 
     * @return Age of the data in milliseconds, or UINT32_MAX if no data available.
     */
    uint32_t get_data_age_ms() const;

    /**
     * @brief Checks if the current sensor data is considered fresh.
     * 
     * @return True if data is within the configured timeout, false otherwise.
     */
    bool is_data_fresh() const;

    /**
     * @brief Forces an update of sensor data by requesting from CAN bus.
     * 
     * This method can be used to actively request sensor data updates
     * instead of waiting for periodic broadcasts.
     * 
     * @return SensorDataResult::SUCCESS on success, error code otherwise.
     */
    SensorDataResult request_data_update();

private:
    /**
     * @brief Callback function for receiving CAN frames.
     * @param frame The received CAN frame.
     */
    void on_can_frame_received(const can::CanFrame& frame);

    /**
     * @brief Updates engine-related sensor data from J1939 message.
     * @param engine_data The decoded engine data from J1939.
     */
    void update_engine_data(const can::j1939::EngineData& engine_data);

    /**
     * @brief Updates vehicle-related sensor data from J1939 message.
     * @param vehicle_data The decoded vehicle data from J1939.
     */
    void update_vehicle_data(const can::j1939::VehicleData& vehicle_data);

    /**
     * @brief Updates CVT status data from J1939 message.
     * @param cvt_status The decoded CVT status from J1939.
     */
    void update_cvt_status(const can::j1939::CvtStatusReport& cvt_status);

    /**
     * @brief Validates and consolidates sensor data into perception data structure.
     */
    void consolidate_perception_data();

    /**
     * @brief Invokes the data callback if set and data is valid.
     */
    void notify_data_callback();

    std::shared_ptr<can::ICanInterface> can_interface_;  ///< CAN interface for communication
    mutable std::mutex data_mutex_;                      ///< Mutex to protect sensor data access
    
    // Raw sensor data from different sources
    can::j1939::EngineData engine_data_;                 ///< Engine-related sensor data
    can::j1939::VehicleData vehicle_data_;               ///< Vehicle-related sensor data
    can::j1939::CvtStatusReport cvt_status_;             ///< CVT status data
    
    // Consolidated perception data
    common::PerceptionData perception_data_;             ///< Consolidated sensor data
    std::chrono::steady_clock::time_point last_update_time_; ///< Timestamp of last data update
    
    // Configuration and state
    uint32_t data_timeout_ms_;                           ///< Data timeout in milliseconds
    bool is_initialized_;                                ///< True if manager is initialized
    SensorDataCallback data_callback_;                   ///< Callback for data updates
};

} // namespace sensors
} // namespace vcu

#endif // SENSOR_DATA_MANAGER_H
