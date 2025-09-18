// include/diagnostic/data_logger.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <fstream>
#include <memory>
#include <queue>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace VCUCore {

// 日志错误信息
struct LogError {
    Timestamp timestamp;
    uint16_t errorCode;
    std::string description;
    enum class ErrorSeverity { LOW, MEDIUM, HIGH } severity;
};

// 日志级别
enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

class DataLogger {
private:
    struct LogConfig {
        std::string directory;
        std::string filenamePattern;
        uint32_t maxFileSizeMB;
        uint32_t maxFiles;
        uint32_t flushIntervalMs;
        std::string compressionLevel;
        bool enableEncryption;
        std::string encryptionKey;
        uint32_t retentionDays;
    };
    
    LogConfig config_;
    std::ofstream currentFile_;
    std::queue<std::string> writeQueue_;
    
    std::atomic<bool> isRunning_;
    std::atomic<uint64_t> bytesWritten_;
    std::atomic<uint32_t> filesCreated_;
    
    std::thread writerThread_;
    std::mutex queueMutex_;
    std::condition_variable queueCondition_;
    
    // 统计信息
    struct LogStatistics {
        uint64_t totalBytes;
        uint64_t totalEntries;
        uint32_t filesCount;
        uint32_t errorsCount;
        float averageWriteTime;
        uint32_t queueSize;
    };
    
    LogStatistics stats_;
    LogLevel currentLogLevel_;

public:
    DataLogger(const std::string& directory = "/var/log/vcu");
    ~DataLogger();
    
    bool initialize(const LogConfig& config);
    bool start();
    bool stop();
    
    // 数据记录
    bool logSensorData(const SensorData& data);
    bool logPerceptionData(const PerceptionData& data);
    bool logControlCommands(const ControlCommands& commands);
    bool logSystemStatus(const SystemHealthStatus& status);
    bool logFaultEvent(const FaultDiagnosis& fault);
    bool logPerformanceData(const PerformanceStatistics& performance);
    
    // 文件管理
    bool rotateLogFile();
    bool compressOldFiles() const;
    bool cleanupOldFiles() const;
    std::vector<std::string> getLogFiles() const;
    
    // 配置管理
    bool setLogLevel(LogLevel level);
    bool setMaxFileSize(uint32_t sizeMB);
    bool setRetentionDays(uint32_t days);
    
    // 状态查询
    LogStatistics getStatistics() const;
    bool isHealthy() const;
    uint32_t getQueueSize() const;
    
    // 诊断功能
    bool verifyLogIntegrity() const;
    std::vector<LogError> getErrors() const;

private:
    void writerThreadFunc();
    bool writeToFile(const std::string& data);
    std::string formatLogEntry(const std::string& type, const std::string& data) const;
    
    std::string generateFilename() const;
    bool openNewFile();
    bool closeCurrentFile();
    
    void updateStatistics(uint64_t bytes, uint32_t entries, float writeTime);
    void handleWriteError(const std::string& error);
    
    // 数据序列化
    std::string serializeSensorData(const SensorData& data) const;
    std::string serializePerceptionData(const PerceptionData& data) const;
    std::string serializeControlCommands(const ControlCommands& commands) const;
    std::string serializeSystemStatus(const SystemHealthStatus& status) const;
    std::string serializeFaultEvent(const FaultDiagnosis& fault) const;
    std::string serializePerformanceData(const PerformanceStatistics& performance) const;
    
    // 数据处理
    std::string compressData(const std::string& data) const;
    std::string encryptData(const std::string& data, const std::string& key) const;
    
    bool shouldRotateFile() const;
    void performAutoRotation();
};

} // namespace VCUCore