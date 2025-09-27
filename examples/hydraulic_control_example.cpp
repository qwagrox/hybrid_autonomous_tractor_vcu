/**
 * @file hydraulic_control_example.cpp
 * @brief Example program demonstrating hydraulic control functionality
 */

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

#include "vcu/core/hydraulic_service.h"
#include "vcu/cvt/cvt_strategy_factory.h"
#include "vcu/can/socketcan_interface.h"
#include "vcu/common/vcu_data_types.h"
#include "vcu/common/vcu_types.h"

// Global flag for graceful shutdown
std::atomic<bool> g_running{true};

void signal_handler(int signal) {
    std::cout << "Received signal " << signal << ", shutting down..." << std::endl;
    g_running = false;
}

/**
 * @brief Callback function for hydraulic state changes
 */
void hydraulic_state_callback(const vcu::common::HydraulicState& state) {
    std::cout << "=== Hydraulic State Update ===" << std::endl;
    std::cout << "Lift Position: " << state.lift_position << "%" << std::endl;
    std::cout << "Lift Mode: " << static_cast<int>(state.lift_mode) << std::endl;
    std::cout << "Lift Moving: " << (state.lift_moving ? "Yes" : "No") << std::endl;
    std::cout << "Shock Absorb Active: " << (state.shock_absorb_active ? "Yes" : "No") << std::endl;
    
    // Display valve states
    for (int i = 0; i < 4; ++i) {
        std::cout << "Valve " << i << " State: " << static_cast<int>(state.valve_states[i]) << std::endl;
    }
    
    std::cout << "Pressure: " << state.pressure << " MPa" << std::endl;
    std::cout << "Temperature: " << state.temperature << "°C" << std::endl;
    std::cout << "System Ready: " << (state.is_ready ? "Yes" : "No") << std::endl;
    std::cout << "Error Code: 0x" << std::hex << state.error_code << std::dec << std::endl;
    std::cout << "==============================" << std::endl;
}

/**
 * @brief Callback function for hydraulic errors
 */
void hydraulic_error_callback(uint32_t error_flags) {
    std::cout << "!!! Hydraulic Error Detected !!!" << std::endl;
    std::cout << "Error Flags: 0x" << std::hex << error_flags << std::dec << std::endl;
    
    // Decode common error flags
    if (error_flags & 0x01) std::cout << "- Low pressure warning" << std::endl;
    if (error_flags & 0x02) std::cout << "- High temperature warning" << std::endl;
    if (error_flags & 0x04) std::cout << "- System not ready" << std::endl;
    if (error_flags & 0x08) std::cout << "- Communication error" << std::endl;
    
    std::cout << "=================================" << std::endl;
}

/**
 * @brief Demonstrate basic lift control operations
 */
void demonstrate_lift_control(std::shared_ptr<vcu::core::HydraulicService> hydraulic_service) {
    std::cout << "\n=== Demonstrating Lift Control ===" << std::endl;
    
    // Set lift mode to manual
    if (hydraulic_service->set_lift_mode(vcu::common::LiftMode::MANUAL)) {
        std::cout << "✓ Set lift mode to MANUAL" << std::endl;
    } else {
        std::cout << "✗ Failed to set lift mode" << std::endl;
        return;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Lift up to 50%
    if (hydraulic_service->set_lift_position(50.0f)) {
        std::cout << "✓ Set lift position to 50%" << std::endl;
    } else {
        std::cout << "✗ Failed to set lift position" << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Execute lift up action
    if (hydraulic_service->execute_lift_action(vcu::common::LiftAction::LIFT_UP)) {
        std::cout << "✓ Executed LIFT_UP action" << std::endl;
    } else {
        std::cout << "✗ Failed to execute lift action" << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Stop lift movement
    if (hydraulic_service->execute_lift_action(vcu::common::LiftAction::STOP)) {
        std::cout << "✓ Stopped lift movement" << std::endl;
    } else {
        std::cout << "✗ Failed to stop lift" << std::endl;
    }
}

/**
 * @brief Demonstrate multi-valve control operations
 */
void demonstrate_multi_valve_control(std::shared_ptr<vcu::core::HydraulicService> hydraulic_service) {
    std::cout << "\n=== Demonstrating Multi-Valve Control ===" << std::endl;
    
    // Test each valve
    for (uint8_t valve_id = 0; valve_id < 4; ++valve_id) {
        std::cout << "Testing valve " << static_cast<int>(valve_id) << "..." << std::endl;
        
        // Set valve to 30% flow (extend)
        if (hydraulic_service->set_multi_valve_flow(valve_id, 30)) {
            std::cout << "✓ Set valve " << static_cast<int>(valve_id) << " to 30% extend" << std::endl;
        } else {
            std::cout << "✗ Failed to set valve " << static_cast<int>(valve_id) << " flow" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // Set valve to neutral
        if (hydraulic_service->set_multi_valve_flow(valve_id, 0)) {
            std::cout << "✓ Set valve " << static_cast<int>(valve_id) << " to neutral" << std::endl;
        } else {
            std::cout << "✗ Failed to set valve " << static_cast<int>(valve_id) << " to neutral" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

/**
 * @brief Demonstrate hydraulic command execution
 */
void demonstrate_hydraulic_commands(std::shared_ptr<vcu::core::HydraulicService> hydraulic_service) {
    std::cout << "\n=== Demonstrating Hydraulic Commands ===" << std::endl;
    
    // Create a hydraulic command for lift up
    vcu::common::HydraulicCommand command;
    command.type = vcu::common::HydraulicCommandType::LIFT_UP;
    command.position = 75.0f;
    command.flow_percent = 50;
    command.valve_id = 0;
    command.valve_lock = false;
    command.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    // Execute the command
    if (hydraulic_service->execute_hydraulic_command(command)) {
        std::cout << "✓ Executed LIFT_UP command successfully" << std::endl;
    } else {
        std::cout << "✗ Failed to execute LIFT_UP command" << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Create a command to stop lift
    command.type = vcu::common::HydraulicCommandType::LIFT_STOP;
    command.position = 0.0f;
    command.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    if (hydraulic_service->execute_hydraulic_command(command)) {
        std::cout << "✓ Executed LIFT_STOP command successfully" << std::endl;
    } else {
        std::cout << "✗ Failed to execute LIFT_STOP command" << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Create a command to set valve flow
    command.type = vcu::common::HydraulicCommandType::SET_VALVE_FLOW;
    command.valve_id = 1;
    command.flow_percent = 40;
    command.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    if (hydraulic_service->execute_hydraulic_command(command)) {
        std::cout << "✓ Executed SET_VALVE_FLOW command successfully" << std::endl;
    } else {
        std::cout << "✗ Failed to execute SET_VALVE_FLOW command" << std::endl;
    }
}

/**
 * @brief Main function
 */
int main(int argc, char* argv[]) {
    std::cout << "=== Hydraulic Control Example ===" << std::endl;
    
    // Set up signal handlers for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    try {
        // Create CAN interface
        auto can_interface = std::make_shared<vcu::can::SocketCanInterface>();
        
        // Initialize CAN interface
        std::string interface_name = (argc > 1) ? argv[1] : "can0";
        uint32_t bitrate = (argc > 2) ? std::stoul(argv[2]) : 250000;
        
        if (can_interface->initialize(interface_name, bitrate) != vcu::can::CanResult::SUCCESS) {
            std::cerr << "Failed to initialize CAN interface: " << interface_name << std::endl;
            return -1;
        }
        
        std::cout << "✓ CAN interface initialized: " << interface_name << " @ " << bitrate << " bps" << std::endl;
        
        // Create CVT strategy using the correct factory function signature
        auto cvt_strategy = vcu::cvt::CvtStrategyFactory::create_strategy(
            vcu::common::CvtManufacturer::HMCVT_VENDOR1,
            *can_interface
        );
        
        if (!cvt_strategy) {
            std::cerr << "Failed to create CVT strategy" << std::endl;
            return -1;
        }
        
        std::cout << "✓ CVT strategy created" << std::endl;
        
        // Create hydraulic service (convert unique_ptr to shared_ptr)
        std::shared_ptr<vcu::cvt::CvtStrategy> shared_strategy(cvt_strategy.release());
        auto hydraulic_service = std::make_shared<vcu::core::HydraulicService>(shared_strategy);
        
        // Register callbacks
        hydraulic_service->register_state_callback(hydraulic_state_callback);
        hydraulic_service->register_error_callback(hydraulic_error_callback);
        
        // Initialize hydraulic service
        if (!hydraulic_service->initialize()) {
            std::cerr << "Failed to initialize hydraulic service" << std::endl;
            return -1;
        }
        
        std::cout << "✓ Hydraulic service initialized" << std::endl;
        
        // Wait for system to be ready
        std::cout << "Waiting for hydraulic system to be ready..." << std::endl;
        int wait_count = 0;
        while (!hydraulic_service->is_hydraulic_ready() && wait_count < 50) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            wait_count++;
        }
        
        if (!hydraulic_service->is_hydraulic_ready()) {
            std::cout << "Warning: Hydraulic system not ready, continuing anyway..." << std::endl;
        } else {
            std::cout << "✓ Hydraulic system ready" << std::endl;
        }
        
        // Create perception data for updates
        vcu::common::PerceptionData perception_data;
        perception_data.vehicle_speed_mps = 1.39f; // 5 km/h in m/s
        perception_data.terrain = vcu::common::TerrainType::FLAT;
        perception_data.load_factor = 0.5f;
        perception_data.accelerator_pedal_percent = 20.0f;
        
        // Main demonstration loop
        std::cout << "\nStarting hydraulic control demonstrations..." << std::endl;
        
        auto last_update = std::chrono::steady_clock::now();
        int demo_step = 0;
        
        while (g_running) {
            auto now = std::chrono::steady_clock::now();
            
            // Update hydraulic service every 100ms
            if (now - last_update >= std::chrono::milliseconds(100)) {
                hydraulic_service->update(perception_data);
                last_update = now;
            }
            
            // Run demonstrations every 10 seconds
            static auto last_demo = std::chrono::steady_clock::now();
            if (now - last_demo >= std::chrono::seconds(10)) {
                switch (demo_step % 3) {
                    case 0:
                        demonstrate_lift_control(hydraulic_service);
                        break;
                    case 1:
                        demonstrate_multi_valve_control(hydraulic_service);
                        break;
                    case 2:
                        demonstrate_hydraulic_commands(hydraulic_service);
                        break;
                }
                demo_step++;
                last_demo = now;
            }
            
            // Sleep for a short time
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Shutdown
        std::cout << "\nShutting down hydraulic service..." << std::endl;
        hydraulic_service->shutdown();
        
        std::cout << "✓ Hydraulic control example completed successfully" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
