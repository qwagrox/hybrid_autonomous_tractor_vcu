// Copyright 2025 Manus AI

#include "vcu/diag/file_diagnostic_monitor.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>

namespace vcu {
namespace diag {

FileDiagnosticMonitor::FileDiagnosticMonitor(const std::string& log_file_path)
    : log_file_path_(log_file_path) {
    
    log_file_.open(log_file_path_, std::ios::app);
    if (!log_file_.is_open()) {
        std::cerr << "Warning: Could not open log file " << log_file_path_ 
                  << ", logging to console only" << std::endl;
    }
}

FileDiagnosticMonitor::~FileDiagnosticMonitor() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void FileDiagnosticMonitor::log(LogLevel level, const std::string& message) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    std::string timestamp = get_timestamp();
    std::string level_str = log_level_to_string(level);
    
    std::stringstream log_entry;
    log_entry << "[" << timestamp << "] [" << level_str << "] " << message;
    
    std::string log_line = log_entry.str();
    
    // Write to file
    write_to_file(log_line);
    
    // Also write to console for important messages
    if (level == LogLevel::ERROR || level == LogLevel::FATAL) {
        std::cerr << log_line << std::endl;
    } else if (level == LogLevel::WARN) {
        std::cout << log_line << std::endl;
    }
}

void FileDiagnosticMonitor::report_fault(int fault_code, const std::string& description) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    // Store active fault
    active_faults_[fault_code] = description;
    
    std::stringstream fault_message;
    fault_message << "FAULT " << fault_code << ": " << description;
    
    std::string timestamp = get_timestamp();
    std::stringstream log_entry;
    log_entry << "[" << timestamp << "] [FAULT] " << fault_message.str();
    
    std::string log_line = log_entry.str();
    
    // Write to file
    write_to_file(log_line);
    
    // Always write faults to console
    std::cerr << log_line << std::endl;
}

std::string FileDiagnosticMonitor::get_timestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    
    return ss.str();
}

std::string FileDiagnosticMonitor::log_level_to_string(LogLevel level) const {
    switch (level) {
        case LogLevel::INFO:  return "INFO ";
        case LogLevel::WARN:  return "WARN ";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default:              return "UNKN ";
    }
}

void FileDiagnosticMonitor::write_to_file(const std::string& log_entry) {
    if (log_file_.is_open()) {
        log_file_ << log_entry << std::endl;
        log_file_.flush(); // Ensure immediate write for important logs
    }
}

} // namespace diag
} // namespace vcu
