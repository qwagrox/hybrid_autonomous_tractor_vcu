// Copyright 2025 Manus AI
// VCU Main Service Entry Point

#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <memory>
#include <thread>
#include <chrono>

#include "vcu/core/vcu_service.h"

// Global VCU service instance
std::unique_ptr<vcu::core::VcuService> g_vcu_service;

// Signal handler for graceful shutdown
void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down VCU service..." << std::endl;
    
    if (g_vcu_service) {
        g_vcu_service->shutdown();
    }
    
    exit(0);
}

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -c <config_file>  Specify configuration file path" << std::endl;
    std::cout << "  -d                Run in daemon mode" << std::endl;
    std::cout << "  -h                Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  " << program_name << " -c /etc/vcu/config.json" << std::endl;
}

int main(int argc, char* argv[]) {
    std::string config_file = "/etc/vcu/vcu_config.json";
    bool daemon_mode = false;
    
    // Parse command line arguments
    int opt;
    while ((opt = getopt(argc, argv, "c:dh")) != -1) {
        switch (opt) {
            case 'c':
                config_file = optarg;
                break;
            case 'd':
                daemon_mode = true;
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }
    
    std::cout << "=== VCU Main Service Starting ===" << std::endl;
    std::cout << "Configuration file: " << config_file << std::endl;
    std::cout << "Daemon mode: " << (daemon_mode ? "enabled" : "disabled") << std::endl;
    
    // Set up signal handlers for graceful shutdown
    signal(SIGINT, signal_handler);   // Ctrl+C
    signal(SIGTERM, signal_handler);  // Termination request
    signal(SIGHUP, signal_handler);   // Hangup
    
    try {
        // Create VCU service instance
        g_vcu_service = std::make_unique<vcu::core::VcuService>();
        
        // Initialize VCU service
        if (!g_vcu_service->initialize(config_file)) {
            std::cerr << "ERROR: Failed to initialize VCU service" << std::endl;
            return 1;
        }
        
        std::cout << "VCU service initialized successfully" << std::endl;
        std::cout << "Starting VCU service..." << std::endl;
        std::cout << "Press Ctrl+C to shutdown..." << std::endl;
        
        // Run VCU service (this will block until shutdown)
        g_vcu_service->run();
        
    } catch (const std::exception& e) {
        std::cerr << "FATAL ERROR: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "VCU service shutdown complete" << std::endl;
    return 0;
}
