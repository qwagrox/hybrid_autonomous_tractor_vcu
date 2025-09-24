// Copyright 2025 Manus AI

#ifndef VCU_DIAG_DIAGNOSTIC_MONITOR_H_
#define VCU_DIAG_DIAGNOSTIC_MONITOR_H_

#include <string>

namespace vcu {
namespace diag {

enum class LogLevel {
    INFO,
    WARN,
    ERROR,
    FATAL
};

class DiagnosticMonitor {
public:
    virtual ~DiagnosticMonitor() = default;

    virtual void log(LogLevel level, const std::string& message) = 0;
    virtual void report_fault(int fault_code, const std::string& description) = 0;
};

} // namespace diag
} // namespace vcu

#endif // VCU_DIAG_DIAGNOSTIC_MONITOR_H_
