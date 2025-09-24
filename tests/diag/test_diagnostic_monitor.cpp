// Copyright 2025 Manus AI

#include <gtest/gtest.h>
#include <fstream>
#include <cstdio>
#include "vcu/diag/file_diagnostic_monitor.h"

namespace vcu {
namespace diag {

class DiagnosticMonitorTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_log_path_ = "/tmp/test_diagnostic.log";
    }

    void TearDown() override {
        std::remove(test_log_path_.c_str());
    }

    std::string read_log_file() {
        std::ifstream log_file(test_log_path_);
        if (!log_file.is_open()) {
            return "";
        }
        
        std::string content((std::istreambuf_iterator<char>(log_file)),
                           std::istreambuf_iterator<char>());
        return content;
    }

    std::string test_log_path_;
};

TEST_F(DiagnosticMonitorTest, BasicLogging) {
    FileDiagnosticMonitor monitor(test_log_path_);
    
    monitor.log(LogLevel::INFO, "Test info message");
    monitor.log(LogLevel::WARN, "Test warning message");
    monitor.log(LogLevel::ERROR, "Test error message");
    
    std::string log_content = read_log_file();
    EXPECT_FALSE(log_content.empty());
    EXPECT_NE(log_content.find("Test info message"), std::string::npos);
    EXPECT_NE(log_content.find("Test warning message"), std::string::npos);
    EXPECT_NE(log_content.find("Test error message"), std::string::npos);
}

TEST_F(DiagnosticMonitorTest, FaultReporting) {
    FileDiagnosticMonitor monitor(test_log_path_);
    
    monitor.report_fault(1001, "Test fault description");
    monitor.report_fault(1002, "Another test fault");
    
    std::string log_content = read_log_file();
    EXPECT_NE(log_content.find("FAULT 1001"), std::string::npos);
    EXPECT_NE(log_content.find("Test fault description"), std::string::npos);
    EXPECT_NE(log_content.find("FAULT 1002"), std::string::npos);
    EXPECT_NE(log_content.find("Another test fault"), std::string::npos);
}

TEST_F(DiagnosticMonitorTest, LogLevels) {
    FileDiagnosticMonitor monitor(test_log_path_);
    
    monitor.log(LogLevel::INFO, "Info message");
    monitor.log(LogLevel::WARN, "Warning message");
    monitor.log(LogLevel::ERROR, "Error message");
    monitor.log(LogLevel::FATAL, "Fatal message");
    
    std::string log_content = read_log_file();
    EXPECT_NE(log_content.find("[INFO ]"), std::string::npos);
    EXPECT_NE(log_content.find("[WARN ]"), std::string::npos);
    EXPECT_NE(log_content.find("[ERROR]"), std::string::npos);
    EXPECT_NE(log_content.find("[FATAL]"), std::string::npos);
}

TEST_F(DiagnosticMonitorTest, TimestampFormat) {
    FileDiagnosticMonitor monitor(test_log_path_);
    
    monitor.log(LogLevel::INFO, "Timestamp test");
    
    std::string log_content = read_log_file();
    
    // Check for timestamp format: [YYYY-MM-DD HH:MM:SS.mmm]
    size_t bracket_pos = log_content.find("[");
    EXPECT_NE(bracket_pos, std::string::npos);
    
    // Should contain date and time elements
    EXPECT_NE(log_content.find("-"), std::string::npos); // Date separators
    EXPECT_NE(log_content.find(":"), std::string::npos); // Time separators
    EXPECT_NE(log_content.find("."), std::string::npos); // Millisecond separator
}

TEST_F(DiagnosticMonitorTest, InvalidLogPath) {
    // Test with invalid log path
    FileDiagnosticMonitor monitor("/invalid/path/test.log");
    
    // Should not crash, should handle gracefully
    monitor.log(LogLevel::INFO, "Test message");
    monitor.report_fault(9999, "Test fault");
    
    // No exception should be thrown
    SUCCEED();
}

} // namespace diag
} // namespace vcu
