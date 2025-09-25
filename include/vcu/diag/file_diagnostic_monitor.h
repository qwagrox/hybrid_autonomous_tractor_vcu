// Copyright 2025 Manus AI

#ifndef VCU_DIAG_FILE_DIAGNOSTIC_MONITOR_H_
#define VCU_DIAG_FILE_DIAGNOSTIC_MONITOR_H_

#include "vcu/diag/diagnostic_monitor.h"
#include <fstream>
#include <mutex>
#include <map>

namespace vcu {
namespace diag {

class FileDiagnosticMonitor : public DiagnosticMonitor {
public:
    explicit FileDiagnosticMonitor(const std::string& log_file_path = "/var/log/vcu.log");
    ~FileDiagnosticMonitor() override;

    void log(LogLevel level, const std::string& message) override;
    void report_fault(int fault_code, const std::string& description) override;

private:
    std::string get_timestamp() const;
    std::string log_level_to_string(LogLevel level) const;
    void write_to_file(const std::string& log_entry);

    std::string log_file_path_;
    std::ofstream log_file_;
    std::mutex log_mutex_;
    
    std::map<int, std::string> active_faults_;
};

} // namespace diag
} // namespace vcu

#endif // VCU_DIAG_FILE_DIAGNOSTIC_MONITOR_H_
