/**
 * @file hydraulic_control_example.cpp
 * @brief Example program demonstrating hydraulic control functionality
 * 
 * This example shows how to use the hydraulic control system in a real
 * application scenario, including initialization, basic operations,
 * and error handling.
 */

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <signal.h>

#include "vcu/core/hydraulic_service.h"
#include "vcu/cvt/hmcvt_vendor1_strategy.h"
#include "vcu/cvt/cvt_strategy_factory.h"
#include "vcu/can/socketcan_interface.h"
#include "vcu/common/vcu_data_types.h"

using namespace vcu;
using namespace vcu::core;
using namespace vcu::cvt;
using namespace vcu::common;
using namespace vcu::can;

// Global variables for signal handling
std::unique_ptr<HydraulicService> g_hydraulic_service;
bool g_running = true;

// Signal handler for graceful shutdown
void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    g_running = false;
    
    if (g_hydraulic_service) {
        g_hydraulic_service->emergency_stop();
        g_hydraulic_service->shutdown();
    }
    
    exit(0);
}

// Callback function for hydraulic state changes
void hydraulic_state_callback(const HydraulicState& state) {
    std::cout << "=== Hydraulic State Update ===" << std::endl;
    std::cout << "Lift Position: " << state.lift_position << "%" << std::endl;
    std::cout << "Lift Mode: " << static_cast<int>(state.lift_mode) << std::endl;
    std::cout << "Lift Moving: " << (state.lift_moving ? "Yes" : "No") << std::endl;
    std::cout << "Main Pressure: " << state.main_pressure_mpa << " MPa" << std::endl;
    std::cout << "HST Temperature: " << state.hst_temp_celsius << "°C" << std::endl;
    std::cout << "Hydraulic Enabled: " << (state.hydraulic_enabled ? "Yes" : "No") << std::endl;
    
    std::cout << "Valve States: ";
    for (int i = 0; i < 4; ++i) {
        std::cout << "[" << i << "]=" << static_cast<int>(state.valve_states[i]) << " ";
    }
    std::cout << std::endl << std::endl;
}

// Callback function for hydraulic errors
void hydraulic_error_callback(uint32_t error_flags) {
    if (error_flags != 0) {
        std::cout << "=== Hydraulic Error ===" << std::endl;
        std::cout << "Error Flags: 0x" << std::hex << error_flags << std::dec << std::endl;
        
        if (error_flags & 0x01) std::cout << "- Lift system error" << std::endl;
        if (error_flags & 0x02) std::cout << "- Main pressure insufficient" << std::endl;
        if (error_flags & 0x04) std::cout << "- Rear axle temperature too high" << std::endl;
        if (error_flags & 0x08) std::cout << "- HST temperature too high" << std::endl;
        if (error_flags & 0x10) std::cout << "- Lift filter fault" << std::endl;
        if (error_flags & 0x20) std::cout << "- Suction filter fault" << std::endl;
        if (error_flags & 0x80000000) std::cout << "- CRITICAL ERROR" << std::endl;
        
        std::cout << std::endl;
    }
}

// Demonstration of basic lift control
void demonstrate_lift_control(HydraulicService& service) {
    std::cout << "\n=== Demonstrating Lift Control ===" << std::endl;
    
    // Set manual mode
    std::cout << "Setting lift mode to MANUAL..." << std::endl;
    if (!service.set_lift_mode(LiftMode::MANUAL)) {
        std::cout << "Failed to set lift mode!" << std::endl;
        return;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Lift up to 25%
    std::cout << "Lifting to 25% position..." << std::endl;
    service.set_lift_position(25.0f);
    service.execute_lift_action(LiftAction::LIFT_UP);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Stop lift
    std::cout << "Stopping lift..." << std::endl;
    service.execute_lift_action(LiftAction::STOP);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Lift to 75%
    std::cout << "Lifting to 75% position..." << std::endl;
    service.set_lift_position(75.0f);
    service.execute_lift_action(LiftAction::LIFT_UP);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Lower to 50%
    std::cout << "Lowering to 50% position..." << std::endl;
    service.set_lift_position(50.0f);
    service.execute_lift_action(LiftAction::LIFT_DOWN);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Stop lift
    std::cout << "Stopping lift..." << std::endl;
    service.execute_lift_action(LiftAction::STOP);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

// Demonstration of multi-valve control
void demonstrate_multi_valve_control(HydraulicService& service) {
    std::cout << "\n=== Demonstrating Multi-Valve Control ===" << std::endl;
    
    // Extend valve 0
    std::cout << "Extending valve 0 at 50% flow..." << std::endl;
    service.set_multi_valve_flow(0, 50);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Retract valve 1
    std::cout << "Retracting valve 1 at 30% flow..." << std::endl;
    service.set_multi_valve_flow(1, -30);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Control multiple valves simultaneously
    std::cout << "Controlling multiple valves simultaneously..." << std::endl;
    service.set_multi_valve_flow(0, 25);   // Extend valve 0 slowly
    service.set_multi_valve_flow(1, -40);  // Retract valve 1 faster
    service.set_multi_valve_flow(2, 60);   // Extend valve 2 fast
    service.set_multi_valve_flow(3, 0);    // Keep valve 3 neutral
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Stop all valves
    std::cout << "Stopping all valves..." << std::endl;
    for (int i = 0; i < 4; ++i) {
        service.set_multi_valve_flow(i, 0);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

// Demonstration of automatic modes
void demonstrate_automatic_modes(HydraulicService& service) {
    std::cout << "\n=== Demonstrating Automatic Modes ===" << std::endl;
    
    // Switch to auto depth mode
    std::cout << "Switching to AUTO_DEPTH mode..." << std::endl;
    service.set_lift_mode(LiftMode::AUTO_DEPTH);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Switch to shock absorb mode
    std::cout << "Switching to SHOCK_ABSORB mode..." << std::endl;
    service.set_lift_mode(LiftMode::SHOCK_ABSORB);
    service.execute_lift_action(LiftAction::SHOCK_ENABLE);
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Disable shock absorb
    std::cout << "Disabling shock absorb..." << std::endl;
    service.execute_lift_action(LiftAction::SHOCK_DISABLE);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Switch back to manual mode
    std::cout << "Switching back to MANUAL mode..." << std::endl;
    service.set_lift_mode(LiftMode::MANUAL);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

// Demonstration of complex hydraulic command
void demonstrate_complex_command(HydraulicService& service) {
    std::cout << "\n=== Demonstrating Complex Hydraulic Command ===" << std::endl;
    
    // Create a complex hydraulic command
    HydraulicCommand command;
    command.lift_action = LiftAction::LIFT_UP;
    command.lift_mode = LiftMode::AUTO_LOAD;
    command.target_position = 60.0f;
    command.target_depth = 40.0f;
    command.lift_speed = 70;
    command.force_position_mix = 80;
    command.upper_limit = 95;
    command.valve_flows[0] = 35;   // Extend valve 0
    command.valve_flows[1] = -25;  // Retract valve 1
    command.valve_flows[2] = 0;    // Neutral valve 2
    command.valve_flows[3] = 45;   // Extend valve 3
    command.valve_lock = false;
    command.hydraulic_enable = true;
    command.emergency_stop = false;
    
    std::cout << "Executing complex hydraulic command..." << std::endl;
    if (service.execute_hydraulic_command(command)) {
        std::cout << "Command executed successfully!" << std::endl;
    } else {
        std::cout << "Failed to execute command!" << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Stop all operations
    std::cout << "Stopping all operations..." << std::endl;
    command.lift_action = LiftAction::STOP;
    for (int i = 0; i < 4; ++i) {
        command.valve_flows[i] = 0;
    }
    service.execute_hydraulic_command(command);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

// Demonstration of safety features
void demonstrate_safety_features(HydraulicService& service) {
    std::cout << "\n=== Demonstrating Safety Features ===" << std::endl;
    
    // Start some operations
    std::cout << "Starting some hydraulic operations..." << std::endl;
    service.set_multi_valve_flow(0, 40);
    service.set_multi_valve_flow(1, -30);
    service.execute_lift_action(LiftAction::LIFT_UP);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Execute emergency stop
    std::cout << "Executing EMERGENCY STOP..." << std::endl;
    if (service.emergency_stop()) {
        std::cout << "Emergency stop executed successfully!" << std::endl;
    } else {
        std::cout << "Failed to execute emergency stop!" << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Lock valves for safety
    std::cout << "Locking all valves for safety..." << std::endl;
    service.set_multi_valve_lock(true);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Unlock valves
    std::cout << "Unlocking valves..." << std::endl;
    service.set_multi_valve_lock(false);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

int main(int argc, char* argv[]) {
    std::cout << "=== Hydraulic Control Example ===" << std::endl;
    std::cout << "This example demonstrates the hydraulic control system functionality." << std::endl;
    std::cout << "Press Ctrl+C to exit at any time." << std::endl << std::endl;
    
    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    try {
        // Create CAN interface (use SocketCAN for real hardware, mock for simulation)
        std::string can_interface_name = "can0"; // Default CAN interface
        if (argc > 1) {
            can_interface_name = argv[1];
        }
        
        std::cout << "Initializing CAN interface: " << can_interface_name << std::endl;
        auto can_interface = std::make_shared<SocketCanInterface>(can_interface_name);
        
        if (!can_interface->initialize()) {
            std::cout << "Warning: Failed to initialize CAN interface. Using simulation mode." << std::endl;
            // In a real application, you might want to use a mock CAN interface here
        }
        
        // Create CVT strategy
        std::cout << "Creating CVT strategy for HMCVT Vendor1..." << std::endl;
        auto cvt_strategy = CvtStrategyFactory::create_strategy(
            CvtManufacturer::HMCVT_VENDOR1, *can_interface);
        
        if (!cvt_strategy) {
            std::cerr << "Failed to create CVT strategy!" << std::endl;
            return 1;
        }
        
        // Initialize CVT strategy
        std::cout << "Initializing CVT strategy..." << std::endl;
        cvt_strategy->init();
        
        // Create hydraulic service
        std::cout << "Creating hydraulic service..." << std::endl;
        g_hydraulic_service = std::make_unique<HydraulicService>(cvt_strategy);
        
        // Register callbacks
        std::cout << "Registering callbacks..." << std::endl;
        g_hydraulic_service->register_state_callback(hydraulic_state_callback);
        g_hydraulic_service->register_error_callback(hydraulic_error_callback);
        
        // Initialize hydraulic service
        std::cout << "Initializing hydraulic service..." << std::endl;
        if (!g_hydraulic_service->initialize()) {
            std::cerr << "Failed to initialize hydraulic service!" << std::endl;
            return 1;
        }
        
        std::cout << "Hydraulic service initialized successfully!" << std::endl;
        std::cout << "System ready: " << (g_hydraulic_service->is_hydraulic_ready() ? "Yes" : "No") << std::endl;
        std::cout << "Hydraulic enabled: " << (g_hydraulic_service->is_hydraulic_enabled() ? "Yes" : "No") << std::endl;
        
        // Wait a moment for system to stabilize
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Run demonstrations
        if (g_running) demonstrate_lift_control(*g_hydraulic_service);
        if (g_running) demonstrate_multi_valve_control(*g_hydraulic_service);
        if (g_running) demonstrate_automatic_modes(*g_hydraulic_service);
        if (g_running) demonstrate_complex_command(*g_hydraulic_service);
        if (g_running) demonstrate_safety_features(*g_hydraulic_service);
        
        // Continuous monitoring loop
        std::cout << "\n=== Entering Monitoring Mode ===" << std::endl;
        std::cout << "Monitoring hydraulic system... (Press Ctrl+C to exit)" << std::endl;
        
        PerceptionData perception_data;
        perception_data.data_valid = true;
        perception_data.vehicle_speed_kmh = 5.0f;
        perception_data.engine_speed_rpm = 1800.0f;
        perception_data.engine_load_percent = 45.0f;
        
        while (g_running) {
            // Update hydraulic service
            g_hydraulic_service->update(perception_data);
            
            // Check system status
            if (!g_hydraulic_service->is_hydraulic_ready()) {
                std::cout << "Warning: Hydraulic system not ready!" << std::endl;
                uint32_t errors = g_hydraulic_service->get_hydraulic_errors();
                if (errors != 0) {
                    std::cout << "Error flags: 0x" << std::hex << errors << std::dec << std::endl;
                }
            }
            
            // Sleep for a short period
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "\nHydraulic control example completed successfully!" << std::endl;
    return 0;
}
