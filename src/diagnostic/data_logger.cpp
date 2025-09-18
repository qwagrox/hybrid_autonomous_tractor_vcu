// src/diagnostic/data_logger.cpp
#include "data_logger.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <filesystem>
#include <zlib.h>
#include <openssl/evp.h>
#include <openssl/err.h>

namespace VCUCore {

DataLogger::DataLogger(const std::string& directory) 
    : isRunning_(false), bytesWritten_(0), filesCreated_(0) {
    
    config_.directory = directory;
    config_.maxFileSizeMB = 100;
    config_.maxFiles = 10;
    config_.flushIntervalMs = 1000;
    config_.compressionLevel = "medium";
    config_.enableEncryption = false;
    
    stats_ = {
        .totalBytes = 0,
        .totalEntries = 0,
        .filesCount = 0,
        .errorsCount = 0,
        .averageWriteTime = 0.0f,
        .queueSize = 0
    };
}

DataLogger::~DataLogger() {
    stop();
}

bool DataLogger::initialize(const LogConfig& config) {
    config_ = config;
    
    // 创建日志目录
    if (!std::filesystem::exists(config_.directory)) {
        if (!std::filesystem::create_directories(config_.directory)) {
            std::cerr << "Failed to create log directory: " << config_.directory << std::endl;
            return false;
        }
    }
    
    // 检查目录可写性
    if (!std::filesystem::is_directory(config_.directory) || 
        access(config_.directory.c_str(), W_OK) != 0) {
        std::cerr << "Log directory is not writable: " << config_.directory << std::endl;
        return false;
    }
    
    // 打开第一个日志文件
    if (!openNewFile()) {
        return false;
    }
    
    std::cout << "Data logger initialized with directory: " << config_.directory << std::endl;
    return true;
}

bool DataLogger::start() {
    if (isRunning_) {
        return true;
    }
    
    isRunning_ = true;
    writerThread_ = std::thread(&DataLogger::writerThreadFunc, this);
    
    std::cout << "Data logger started" << std::endl;
    return true;
}

bool DataLogger::stop() {
    if (!isRunning_) {
        return true;
    }
    
    isRunning_ = false;
    queueCondition_.notify_all();
    
    if (writerThread_.joinable()) {
        writerThread_.join();
    }
    
    closeCurrentFile();
    
    std::cout << "Data logger stopped. Total entries: " << stats_.totalEntries << std::endl;
    return true;
}

void DataLogger::writerThreadFunc() {
    while (isRunning_) {
        std::unique_lock<std::mutex> lock(queueMutex_);
        
        if (writeQueue_.empty()) {
            queueCondition_.wait_for(lock, std::chrono::milliseconds(config_.flushIntervalMs));
            continue;
        }
        
        // 获取要写入的数据
        std::string data = writeQueue_.front();
        writeQueue_.pop();
        stats_.queueSize = writeQueue_.size();
        
        lock.unlock();
        
        // 写入文件
        auto startTime = std::chrono::high_resolution_clock::now();
        bool success = writeToFile(data);
        auto endTime = std::chrono::high_resolution_clock::now();
        
        float writeTime = std::chrono::duration_cast<std::chrono::microseconds>(
            endTime - startTime).count() / 1000.0f;
        
        if (success) {
            updateStatistics(data.size(), 1, writeTime);
        } else {
            stats_.errorsCount++;
            handleWriteError("Failed to write to log file");
        }
        
        // 检查文件轮转
        if (shouldRotateFile()) {
            performAutoRotation();
        }
    }
}

bool DataLogger::writeToFile(const std::string& data) {
    if (!currentFile_.is_open()) {
        if (!openNewFile()) {
            return false;
        }
    }
    
    try {
        std::string processedData = data;
        
        // 应用压缩（如果需要）
        if (config_.compressionLevel != "none") {
            processedData = compressData(processedData);
        }
        
        // 应用加密（如果需要）
        if (config_.enableEncryption && !config_.encryptionKey.empty()) {
            processedData = encryptData(processedData, config_.encryptionKey);
        }
        
        currentFile_ << processedData << std::endl;
        bytesWritten_ += processedData.size();
        
        // 定期刷新
        if (bytesWritten_ % 4096 == 0) {
            currentFile_.flush();
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Write error: " << e.what() << std::endl;
        return false;
    }
}

bool DataLogger::logSensorData(const SensorData& data) {
    std::string serialized = serializeSensorData(data);
    std::string entry = formatLogEntry("SENSOR", serialized);
    
    std::lock_guard<std::mutex> lock(queueMutex_);
    writeQueue_.push(entry);
    queueCondition_.notify_one();
    
    return true;
}

bool DataLogger::logPerceptionData(const PerceptionData& data) {
    std::string serialized = serializePerceptionData(data);
    std::string entry = formatLogEntry("PERCEPTION", serialized);
    
    std::lock_guard<std::mutex> lock(queueMutex_);
    writeQueue_.push(entry);
    queueCondition_.notify_one();
    
    return true;
}

bool DataLogger::logControlCommands(const ControlCommands& commands) {
    std::string serialized = serializeControlCommands(commands);
    std::string entry = formatLogEntry("CONTROL", serialized);
    
    std::lock_guard<std::mutex> lock(queueMutex_);
    writeQueue_.push(entry);
    queueCondition_.notify_one();
    
    return true;
}

bool DataLogger::logSystemStatus(const SystemHealthStatus& status) {
    std::string serialized = serializeSystemStatus(status);
    std::string entry = formatLogEntry("HEALTH", serialized);
    
    std::lock_guard<std::mutex> lock(queueMutex_);
    writeQueue_.push(entry);
    queueCondition_.notify_one();
    
    return true;
}

bool DataLogger::logFaultEvent(const FaultDiagnosis& fault) {
    std::string serialized = serializeFaultEvent(fault);
    std::string entry = formatLogEntry("FAULT", serialized);
    
    std::lock_guard<std::mutex> lock(queueMutex_);
    writeQueue_.push(entry);
    queueCondition_.notify_one();
    
    return true;
}

bool DataLogger::logPerformanceData(const PerformanceStatistics& performance) {
    std::string serialized = serializePerformanceData(performance);
    std::string entry = formatLogEntry("PERFORMANCE", serialized);
    
    std::lock_guard<std::mutex> lock(queueMutex_);
    writeQueue_.push(entry);
    queueCondition_.notify_one();
    
    return true;
}

std::string DataLogger::serializeSensorData(const SensorData& data) const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    
    ss << "timestamp=" << data.timestamp.count() << ",";
    ss << "gps_x=" << data.gnssPosition.x() << ",";
    ss << "gps_y=" << data.gnssPosition.y() << ",";
    ss << "gps_z=" << data.gnssPosition.z() << ",";
    ss << "velocity_x=" << data.gnssVelocity.x() << ",";
    ss << "velocity_y=" << data.gnssVelocity.y() << ",";
    ss << "velocity_z=" << data.gnssVelocity.z() << ",";
    ss << "accel_x=" << data.imuAcceleration.x() << ",";
    ss << "accel_y=" << data.imuAcceleration.y() << ",";
    ss << "accel_z=" << data.imuAcceleration.z() << ",";
    ss << "gyro_x=" << data.imuAngularRate.x() << ",";
    ss << "gyro_y=" << data.imuAngularRate.y() << ",";
    ss << "gyro_z=" << data.imuAngularRate.z() << ",";
    ss << "engine_rpm=" << data.engineRpm << ",";
    ss << "motor_rpm=" << data.motorRpm << ",";
    ss << "battery_voltage=" << data.batteryVoltage << ",";
    ss << "battery_current=" << data.batteryCurrent << ",";
    ss << "battery_soc=" << data.batterySOC << ",";
    ss << "battery_temp=" << data.batteryTemperature << ",";
    ss << "engine_temp=" << data.engineTemperature << ",";
    ss << "motor_temp=" << data.motorTemperature << ",";
    ss << "hydraulic_pressure=" << data.hydraulicPressure << ",";
    ss << "implement_depth=" << data.implementDepth << ",";
    ss << "implement_force=" << data.implementForce << ",";
    ss << "soil_moisture=" << data.soilMoisture << ",";
    ss << "soil_compaction=" << data.soilCompaction << ",";
    ss << "fuel_rate=" << data.fuelRate << ",";
    ss << "ambient_temp=" << data.ambientTemperature << ",";
    ss << "ambient_humidity=" << data.ambientHumidity << ",";
    ss << "wind_speed=" << data.windSpeed;
    
    return ss.str();
}

std::string DataLogger::serializePerceptionData(const PerceptionData& data) const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    
    ss << "timestamp=" << data.timestamp << ",";
    ss << "position_x=" << data.vehicleState.position.x() << ",";
    ss << "position_y=" << data.vehicleState.position.y() << ",";
    ss << "position_z=" << data.vehicleState.position.z() << ",";
    ss << "velocity=" << data.vehicleState.velocity.norm() << ",";
    ss << "acceleration=" << data.vehicleState.acceleration.norm() << ",";
    ss << "heading=" << data.vehicleState.heading << ",";
    ss << "pitch=" << data.vehicleState.pitch << ",";
    ss << "roll=" << data.vehicleState.roll << ",";
    ss << "mass=" << data.vehicleState.estimatedMass << ",";
    ss << "drawbar_pull=" << data.vehicleState.drawbarPull << ",";
    ss << "slope=" << data.terrainSlope << ",";
    ss << "soil_resistance=" << data.soilResistance << ",";
    ss << "rolling_resistance=" << data.rollingResistance << ",";
    ss << "aerodynamic_drag=" << data.aerodynamicDrag << ",";
    ss << "load_factor=" << data.loadFactor << ",";
    ss << "load_change_type=" << static_cast<int>(data.loadChangeType) << ",";
    ss << "load_trend=" << static_cast<int>(data.loadTrend) << ",";
    ss << "confidence=" << data.confidence << ",";
    ss << "stability_index=" << data.stabilityIndex << ",";
    ss << "traction_efficiency=" << data.tractionEfficiency;
    
    return ss.str();
}

std::string DataLogger::serializeControlCommands(const ControlCommands& commands) const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    
    ss << "timestamp=" << commands.timestamp << ",";
    ss << "engine_torque=" << commands.engineTorqueRequest << ",";
    ss << "motor_torque=" << commands.motorTorqueRequest << ",";
    ss << "transmission_gear=" << commands.transmissionGearRequest << ",";
    ss << "hydraulic_pressure=" << commands.hydraulicPressureRequest << ",";
    ss << "implement_lift=" << (commands.implementLiftRequest ? "true" : "false") << ",";
    ss << "cvt_ratio=" << commands.cvtRatioRequest << ",";
    ss << "emergency_stop=" << (commands.emergencyStop ? "true" : "false") << ",";
    ss << "control_mode=" << static_cast<int>(commands.controlMode) << ",";
    ss << "max_torque_limit=" << commands.maxTorqueLimit << ",";
    ss << "min_torque_limit=" << commands.minTorqueLimit << ",";
    ss << "torque_change_rate=" << commands.torqueChangeRate << ",";
    ss << "ratio_change_rate=" << commands.ratioChangeRate;
    
    return ss.str();
}

std::string DataLogger::serializeSystemStatus(const SystemHealthStatus& status) const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    
    ss << "timestamp=" << status.timestamp << ",";
    ss << "is_healthy=" << (status.isHealthy ? "true" : "false") << ",";
    ss << "overall_health=" << status.overallHealth << ",";
    ss << "uptime=" << status.uptime << ",";
    ss << "last_maintenance=" << status.lastMaintenance << ",";
    ss << "next_maintenance=" << status.nextMaintenance;
    
    // 添加组件健康状态
    for (const auto& [component, health] : status.componentHealth) {
        ss << "," << component << "_health=" << health;
    }
    
    // 添加活动故障计数
    ss << ",active_faults=" << status.activeFaults.size();
    
    return ss.str();
}

std::string DataLogger::serializeFaultEvent(const FaultDiagnosis& fault) const {
    std::stringstream ss;
    
    ss << "timestamp=" << fault.timestamp << ",";
    ss << "fault_code=" << fault.faultCode << ",";
    ss << "severity=" << static_cast<int>(fault.severity) << ",";
    ss << "description=" << fault.description << ",";
    ss << "component=" << fault.component << ",";
    ss << "duration=" << fault.duration << ",";
    ss << "is_active=" << (fault.isActive ? "true" : "false") << ",";
    ss << "is_recoverable=" << (fault.isRecoverable ? "true" : "false");
    
    // 添加恢复步骤
    if (!fault.recoverySteps.empty()) {
        ss << ",recovery_steps=";
        for (size_t i = 0; i < fault.recoverySteps.size(); ++i) {
            if (i > 0) ss << ";";
            ss << fault.recoverySteps[i];
        }
    }
    
    return ss.str();
}

std::string DataLogger::serializePerformanceData(const PerformanceStatistics& performance) const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    
    ss << "timestamp=" << performance.timestamp << ",";
    ss << "total_cycles=" << performance.totalCycles << ",";
    ss << "missed_deadlines=" << performance.missedDeadlines << ",";
    ss << "avg_cycle_time=" << performance.averageCycleTime << ",";
    ss << "max_cycle_time=" << performance.maxCycleTime << ",";
    ss << "min_cycle_time=" << performance.minCycleTime << ",";
    ss << "cpu_usage=" << performance.cpuUsage << ",";
    ss << "memory_usage=" << performance.memoryUsage << ",";
    ss << "comm_latency=" << performance.communicationLatency << ",";
    ss << "control_accuracy=" << performance.controlAccuracy << ",";
    ss << "energy_efficiency=" << performance.energyEfficiency;
    
    return ss.str();
}

std::string DataLogger::formatLogEntry(const std::string& type, const std::string& data) const {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << milliseconds;
    ss << " [" << type << "] " << data;
    
    return ss.str();
}

bool DataLogger::openNewFile() {
    closeCurrentFile();
    
    std::string filename = generateFilename();
    std::string fullpath = config_.directory + "/" + filename;
    
    try {
        currentFile_.open(fullpath, std::ios::out | std::ios::app);
        if (!currentFile_.is_open()) {
            throw std::runtime_error("Cannot open log file: " + fullpath);
        }
        
        filesCreated_++;
        bytesWritten_ = 0;
        
        // 写入文件头
        currentFile_ << "# VCU System Log File" << std::endl;
        currentFile_ << "# Created: " << std::time(nullptr) << std::endl;
        currentFile_ << "# Version: 1.0" << std::endl;
        currentFile_ << "# Compression: " << config_.compressionLevel << std::endl;
        currentFile_ << "# Encryption: " << (config_.enableEncryption ? "enabled" : "disabled") << std::endl;
        currentFile_ << "########################################" << std::endl;
        
        std::cout << "Opened new log file: " << fullpath << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error opening log file: " << e.what() << std::endl;
        return false;
    }
}

bool DataLogger::closeCurrentFile() {
    if (currentFile_.is_open()) {
        try {
            // 写入文件尾
            currentFile_ << "########################################" << std::endl;
            currentFile_ << "# Closed: " << std::time(nullptr) << std::endl;
            currentFile_ << "# Total entries: " << stats_.totalEntries << std::endl;
            currentFile_ << "# Total bytes: " << bytesWritten_ << std::endl;
            currentFile_.close();
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error closing log file: " << e.what() << std::endl;
            return false;
        }
    }
    return true;
}

std::string DataLogger::generateFilename() const {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "vcu_log_%Y%m%d_%H%M%S");
    ss << "_" << std::setfill('0') << std::setw(4) << filesCreated_ << ".log";
    
    return ss.str();
}

bool DataLogger::shouldRotateFile() const {
    // 检查文件大小是否超过限制
    uint64_t maxBytes = config_.maxFileSizeMB * 1024 * 1024;
    return bytesWritten_ >= maxBytes;
}

void DataLogger::performAutoRotation() {
    std::cout << "Performing log file rotation..." << std::endl;
    
    // 关闭当前文件
    closeCurrentFile();
    
    // 打开新文件
    if (!openNewFile()) {
        std::cerr << "Failed to rotate log file" << std::endl;
        return;
    }
    
    // 清理旧文件
    cleanupOldFiles();
}

bool DataLogger::cleanupOldFiles() const {
    try {
        std::vector<std::filesystem::path> files;
        
        // 获取所有日志文件
        for (const auto& entry : std::filesystem::directory_iterator(config_.directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".log") {
                files.push_back(entry.path());
            }
        }
        
        // 按修改时间排序
        std::sort(files.begin(), files.end(), [](const auto& a, const auto& b) {
            return std::filesystem::last_write_time(a) < std::filesystem::last_write_time(b);
        });
        
        // 删除最旧的文件
        if (files.size() > config_.maxFiles) {
            size_t filesToDelete = files.size() - config_.maxFiles;
            for (size_t i = 0; i < filesToDelete; ++i) {
                std::filesystem::remove(files[i]);
                std::cout << "Deleted old log file: " << files[i] << std::endl;
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error cleaning up old files: " << e.what() << std::endl;
        return false;
    }
}

std::string DataLogger::compressData(const std::string& data) const {
    // 简单的压缩实现 - 实际生产环境应该使用更高效的算法
    if (config_.compressionLevel == "low") {
        // 基础压缩：移除多余空格
        std::string compressed;
        compressed.reserve(data.size());
        
        bool lastWasSpace = false;
        for (char c : data) {
            if (std::isspace(c)) {
                if (!lastWasSpace) {
                    compressed.push_back(' ');
                    lastWasSpace = true;
                }
            } else {
                compressed.push_back(c);
                lastWasSpace = false;
            }
        }
        
        return compressed;
    }
    
    // 中等或高级压缩使用zlib
    if (config_.compressionLevel == "medium" || config_.compressionLevel == "high") {
        z_stream stream;
        stream.zalloc = Z_NULL;
        stream.zfree = Z_NULL;
        stream.opaque = Z_NULL;
        
        int level = config_.compressionLevel == "high" ? Z_BEST_COMPRESSION : Z_DEFAULT_COMPRESSION;
        if (deflateInit(&stream, level) != Z_OK) {
            return data; // 压缩失败，返回原始数据
        }
        
        stream.next_in = (Bytef*)data.data();
        stream.avail_in = data.size();
        
        std::string compressed;
        compressed.resize(data.size() * 1.1 + 12); // 预分配空间
        
        do {
            stream.next_out = (Bytef*)(compressed.data() + stream.total_out);
            stream.avail_out = compressed.size() - stream.total_out;
            
            deflate(&stream, Z_FINISH);
        } while (stream.avail_out == 0);
        
        deflateEnd(&stream);
        compressed.resize(stream.total_out);
        
        return compressed;
    }
    
    return data;
}

std::string DataLogger::encryptData(const std::string& data, const std::string& key) const {
    if (key.empty()) {
        return data;
    }
    
    // 简单的加密实现 - 实际生产环境应该使用更安全的加密方案
    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        return data;
    }
    
    // 使用AES-256-CBC加密
    unsigned char iv[16] = {0}; // 实际应该使用随机IV
    std::string encrypted;
    encrypted.resize(data.size() + 16); // 预留空间
    
    int len;
    int ciphertext_len;
    
    if (EVP_EncryptInit_ex(ctx, EVP_aes_256_cbc(), NULL, 
                          (const unsigned char*)key.c_str(), iv) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        return data;
    }
    
    if (EVP_EncryptUpdate(ctx, (unsigned char*)encrypted.data(), &len,
                         (const unsigned char*)data.data(), data.size()) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        return data;
    }
    ciphertext_len = len;
    
    if (EVP_EncryptFinal_ex(ctx, (unsigned char*)encrypted.data() + len, &len) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        return data;
    }
    ciphertext_len += len;
    
    EVP_CIPHER_CTX_free(ctx);
    encrypted.resize(ciphertext_len);
    
    return encrypted;
}

void DataLogger::updateStatistics(uint64_t bytes, uint32_t entries, float writeTime) {
    stats_.totalBytes += bytes;
    stats_.totalEntries += entries;
    stats_.filesCount = filesCreated_;
    
    // 更新平均写入时间（指数移动平均）
    stats_.averageWriteTime = 0.9f * stats_.averageWriteTime + 0.1f * writeTime;
}

void DataLogger::handleWriteError(const std::string& error) {
    std::cerr << "Write error: " << error << std::endl;
    
    // 尝试恢复：关闭当前文件并打开新文件
    closeCurrentFile();
    if (!openNewFile()) {
        std::cerr << "Failed to recover from write error" << std::endl;
    }
}

LogStatistics DataLogger::getStatistics() const {
    std::lock_guard<std::mutex> lock(queueMutex_);
    return stats_;
}

bool DataLogger::isHealthy() const {
    return stats_.errorsCount == 0 && 
           stats_.queueSize < 1000 && 
           currentFile_.is_open();
}

uint32_t DataLogger::getQueueSize() const {
    std::lock_guard<std::mutex> lock(queueMutex_);
    return stats_.queueSize;
}

bool DataLogger::verifyLogIntegrity() const {
    try {
        // 检查日志目录是否存在且可写
        if (!std::filesystem::exists(config_.directory) ||
            !std::filesystem::is_directory(config_.directory)) {
            return false;
        }
        
        // 检查当前文件是否正常
        if (!currentFile_.is_open()) {
            return false;
        }
        
        // 检查写入队列大小
        if (stats_.queueSize > 5000) {
            return false;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Log integrity check failed: " << e.what() << std::endl;
        return false;
    }
}

std::vector<LogError> DataLogger::getErrors() const {
    std::vector<LogError> errors;
    
    // 这里可以返回具体的错误信息
    // 简化实现：只返回错误计数
    
    if (stats_.errorsCount > 0) {
        errors.push_back({
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch()),
            .errorCode = 0x5001,
            .description = "Write errors detected: " + std::to_string(stats_.errorsCount),
            .severity = stats_.errorsCount > 10 ? ErrorSeverity::HIGH : ErrorSeverity::MEDIUM
        });
    }
    
    if (stats_.queueSize > 1000) {
        errors.push_back({
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch()),
            .errorCode = 0x5002,
            .description = "Write queue overload: " + std::to_string(stats_.queueSize),
            .severity = ErrorSeverity::HIGH
        });
    }
    
    return errors;
}

bool DataLogger::rotateLogFile() {
    std::lock_guard<std::mutex> lock(queueMutex_);
    return performAutoRotation();
}

bool DataLogger::setLogLevel(LogLevel level) {
    // 设置日志级别过滤
    // 简化实现：只在内存中设置
    currentLogLevel_ = level;
    return true;
}

bool DataLogger::setMaxFileSize(uint32_t sizeMB) {
    config_.maxFileSizeMB = sizeMB;
    return true;
}

bool DataLogger::setRetentionDays(uint32_t days) {
    config_.retentionDays = days;
    return true;
}

std::vector<std::string> DataLogger::getLogFiles() const {
    std::vector<std::string> files;
    
    try {
        for (const auto& entry : std::filesystem::directory_iterator(config_.directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".log") {
                files.push_back(entry.path().filename());
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error getting log files: " << e.what() << std::endl;
    }
    
    return files;
}

} // namespace VCUCore