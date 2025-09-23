#include "vcu/cvt/cvt_controller.h"
#include "vcu/can/can_interface.h"
#include "vcu/sensors/sensor_data_manager.h"
#include "vcu/prediction/load_predictor.h"
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

/**
 * @brief Integration example demonstrating the complete VCU system workflow.
 * 
 * This example shows how all modules work together:
 * 1. CAN communication for sensor data
 * 2. Sensor data management and processing
 * 3. Load prediction based on historical data
 * 4. CVT control based on predictions and current conditions
 */

using namespace vcu;

int main() {
    std::cout << "VCU Integration Example\n";
    std::cout << "======================\n\n";

    try {
        // 1. Initialize CAN interface (mock for example)
        auto can_interface = can::create_can_interface("mock");
        if (!can_interface) {
            std::cerr << "Failed to create CAN interface\n";
            return -1;
        }

        std::cout << "1. CAN Interface initialized\n";

        // 2. Initialize sensor data manager
        auto sensor_manager = std::make_unique<sensors::SensorDataManager>(can_interface);
        auto sensor_result = sensor_manager->initialize(1000);
        if (sensor_result != sensors::SensorDataResult::SUCCESS) {
            std::cerr << "Failed to initialize sensor manager\n";
            return -1;
        }

        std::cout << "2. Sensor Data Manager initialized\n";

        // 3. Initialize load predictor
        prediction::LoadPredictionConfig pred_config;
        pred_config.history_window_size = 10;
        pred_config.terrain_weight = 0.3f;
        pred_config.speed_weight = 0.4f;
        pred_config.load_weight = 0.3f;

        auto load_predictor = std::make_unique<prediction::LoadPredictor>(pred_config);
        auto pred_result = load_predictor->initialize();
        if (pred_result != prediction::PredictionResult::SUCCESS) {
            std::cerr << "Failed to initialize load predictor\n";
            return -1;
        }

        std::cout << "3. Load Predictor initialized\n";

        // 4. Initialize CVT controller
        cvt::CvtControllerConfig cvt_config;
        cvt_config.manufacturer = common::CvtManufacturer::BOSCH;
        cvt_config.min_ratio = 0.5f;
        cvt_config.max_ratio = 4.0f;

        auto cvt_controller = std::make_unique<cvt::CvtController>(cvt_config);
        auto cvt_result = cvt_controller->initialize();
        if (cvt_result != cvt::CvtResult::SUCCESS) {
            std::cerr << "Failed to initialize CVT controller\n";
            return -1;
        }

        std::cout << "4. CVT Controller initialized\n\n";

        // 5. Simulate system operation
        std::cout << "Simulating system operation...\n";

        // Simulate some sensor data updates
        for (int i = 0; i < 5; ++i) {
            // Create mock sensor data
            common::PerceptionData sensor_data;
            sensor_data.vehicle_speed_mps = 10.0f + i * 2.0f;
            sensor_data.engine_speed_rpm = 1500.0f + i * 100.0f;
            sensor_data.engine_load_percent = 50.0f + i * 5.0f;
            sensor_data.accelerator_pedal_percent = 60.0f + i * 3.0f;
            sensor_data.terrain_type = (i % 2 == 0) ? common::TerrainType::SMOOTH : common::TerrainType::MODERATE;
            sensor_data.data_valid = true;

            // Update predictor with sensor data
            load_predictor->update_data(sensor_data);

            // Get prediction if available
            common::PredictionResult prediction;
            if (load_predictor->predict_load(prediction) == prediction::PredictionResult::SUCCESS) {
                std::cout << "Cycle " << (i + 1) << ":\n";
                std::cout << "  Current Load: " << sensor_data.engine_load_percent << "%\n";
                std::cout << "  Predicted Load: " << prediction.predicted_load_percent << "%\n";

                // Update CVT controller with current data and prediction
                cvt_controller->update_sensor_data(sensor_data);
                cvt_controller->update_prediction_data(prediction);

                // Calculate optimal transmission ratio
                float optimal_ratio;
                auto control_result = cvt_controller->calculate_optimal_ratio(optimal_ratio);
                if (control_result == cvt::CvtResult::SUCCESS) {
                    std::cout << "  Optimal Ratio: " << optimal_ratio << "\n";

                    // Apply the ratio (in real system, this would send CAN commands)
                    cvt_controller->set_target_ratio(optimal_ratio);
                }
            } else {
                std::cout << "Cycle " << (i + 1) << ": Insufficient data for prediction\n";
            }

            std::cout << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Integration example completed successfully!\n";
        std::cout << "\nSystem Statistics:\n";
        std::cout << "- Historical data points: " << load_predictor->get_history_size() << "\n";
        std::cout << "- CVT ready: " << (cvt_controller->is_ready() ? "Yes" : "No") << "\n";
        std::cout << "- Sensor manager ready: " << (sensor_manager->is_ready() ? "Yes" : "No") << "\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return -1;
    }
}
