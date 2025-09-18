// plow_resistance_simulation.cpp
// 犁地阻力测试用例完整仿真程序

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <deque>
#include <string>
#include <sstream>

// 系统状态结构
struct SystemState {
    double time;                    // 仿真时间 (s)
    double soil_resistance_factor;  // 土壤阻力系数
    double engine_speed;            // 发动机转速 (RPM)
    double engine_torque;           // 发动机扭矩 (Nm)
    double engine_throttle;         // 油门开度 (0-1)
    double motor_torque;            // 电机扭矩 (Nm)
    double motor_speed;             // 电机转速 (RPM)
    double battery_soc;             // 电池SOC (%)
    double cvt_ratio;               // CVT传动比
    double vehicle_speed;           // 车速 (km/h)
    double plow_depth_target;       // 目标犁地深度 (m)
    double plow_depth_actual;       // 实际犁地深度 (m)
    double plow_load_torque;        // 犁具负载扭矩 (Nm)
    double fuel_consumption_rate;   // 燃油消耗率 (L/h)
    int vcu_decision_state;         // VCU决策状态
    std::string system_warnings;   // 系统警告
};

// VCU决策状态枚举
enum class VCUDecisionState {
    NORMAL_OPERATION = 0,      // 正常作业
    DETECTING_ANOMALY = 1,     // 检测异常
    EMERGENCY_RESPONSE = 2,    // 紧急响应
    ADAPTIVE_ADJUSTMENT = 3,   // 自适应调整
    RECOVERY_MODE = 4,         // 恢复模式
    OPTIMIZATION_MODE = 5      // 优化模式
};

// 异常检测器
class AnomalyDetector {
private:
    double load_threshold_absolute_ = 150.0;    // Nm
    double load_rate_threshold_ = 50.0;         // Nm/s
    double detection_window_ = 0.5;             // s
    
    std::deque<double> load_history_;
    std::deque<double> time_history_;
    
public:
    bool detectAnomaly(double current_load, double current_time) {
        // 更新历史数据
        load_history_.push_back(current_load);
        time_history_.push_back(current_time);
        
        // 保持窗口大小
        while (time_history_.size() > 0 && 
               current_time - time_history_.front() > detection_window_) {
            load_history_.pop_front();
            time_history_.pop_front();
        }
        
        if (load_history_.size() < 2) return false;
        
        // 检测1: 绝对负载阈值
        bool absolute_threshold = current_load > load_threshold_absolute_;
        
        // 检测2: 负载变化率
        double load_rate = (load_history_.back() - load_history_.front()) / 
                          (time_history_.back() - time_history_.front());
        bool rate_threshold = load_rate > load_rate_threshold_;
        
        // 检测3: 趋势分析
        bool trend_analysis = analyzeTrend();
        
        return absolute_threshold && (rate_threshold || trend_analysis);
    }
    
private:
    bool analyzeTrend() {
        if (load_history_.size() < 5) return false;
        
        // 简单线性回归检测上升趋势
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        int n = load_history_.size();
        
        for (int i = 0; i < n; i++) {
            sum_x += i;
            sum_y += load_history_[i];
            sum_xy += i * load_history_[i];
            sum_x2 += i * i;
        }
        
        double slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
        return slope > 20.0; // 上升趋势阈值
    }
};

// 多目标优化器
class MultiObjectiveOptimizer {
private:
    struct OptimizationWeights {
        double equipment_protection = 0.4;    // 设备保护权重
        double work_efficiency = 0.3;         // 作业效率权重  
        double fuel_economy = 0.2;            // 燃油经济性权重
        double work_quality = 0.1;            // 作业质量权重
    } weights_;
    
public:
    struct ControlParameters {
        double engine_torque_ratio;    // 发动机扭矩比例 (0.6-1.0)
        double cvt_ratio;              // CVT传动比 (0.8-2.0)
        double plow_depth_ratio;       // 犁地深度比例 (0.3-1.0)
        double vehicle_speed_ratio;    // 车速比例 (0.5-1.0)
        double motor_assist_ratio;     // 电机辅助比例 (0.0-0.6)
    };
    
    ControlParameters optimize(double current_load, double target_load) {
        ControlParameters params;
        
        // 计算负载比例
        double load_ratio = current_load / target_load;
        
        if (load_ratio > 2.0) {
            // 高负载情况：优先保护设备
            params = optimizeForProtection(load_ratio);
        } else if (load_ratio > 1.5) {
            // 中等负载：平衡效率和保护
            params = optimizeForBalance(load_ratio);
        } else {
            // 正常负载：优化效率
            params = optimizeForEfficiency(load_ratio);
        }
        
        return params;
    }
    
private:
    ControlParameters optimizeForProtection(double load_ratio) {
        ControlParameters params;
        
        // 设备保护优先策略
        params.engine_torque_ratio = std::min(0.95, 0.6 + 0.2 * (load_ratio - 1.0));
        params.cvt_ratio = std::max(0.8, 2.0 - 0.3 * load_ratio);
        params.plow_depth_ratio = std::max(0.4, 1.2 - 0.2 * load_ratio);
        params.vehicle_speed_ratio = std::max(0.6, 1.1 - 0.15 * load_ratio);
        params.motor_assist_ratio = std::min(0.5, 0.1 * (load_ratio - 1.5));
        
        return params;
    }
    
    ControlParameters optimizeForBalance(double load_ratio) {
        ControlParameters params;
        
        // 平衡策略
        params.engine_torque_ratio = 0.7 + 0.15 * (load_ratio - 1.0);
        params.cvt_ratio = 1.8 - 0.2 * load_ratio;
        params.plow_depth_ratio = 1.0 - 0.1 * (load_ratio - 1.0);
        params.vehicle_speed_ratio = 1.0 - 0.1 * (load_ratio - 1.0);
        params.motor_assist_ratio = 0.05 * (load_ratio - 1.2);
        
        return params;
    }
    
    ControlParameters optimizeForEfficiency(double load_ratio) {
        ControlParameters params;
        
        // 效率优先策略
        params.engine_torque_ratio = 0.65 + 0.1 * load_ratio;
        params.cvt_ratio = 1.5;
        params.plow_depth_ratio = 1.0;
        params.vehicle_speed_ratio = 1.0;
        params.motor_assist_ratio = 0.0;
        
        return params;
    }
};

// 自适应恢复控制器
class AdaptiveRecoveryController {
private:
    struct RecoveryTargets {
        double target_depth = 0.25;        // m
        double target_speed = 8.0;          // km/h
        double target_cvt_ratio = 1.5;
        double target_engine_load = 0.65;
    } recovery_targets_;
    
    struct RecoveryRates {
        double depth_rate = 0.01;      // m/s
        double speed_rate = 0.5;       // km/h/s
        double cvt_rate = 0.1;         // ratio/s
        double engine_rate = 0.05;     // load%/s
        double motor_rate = 10.0;      // Nm/s
    } recovery_rates_;
    
public:
    void updateRecovery(MultiObjectiveOptimizer::ControlParameters& current, 
                       double dt, double current_load) {
        // 检查是否可以开始恢复
        if (!canStartRecovery(current_load)) {
            return;
        }
        
        // 渐进式恢复各参数
        recoverDepth(current, dt);
        recoverSpeed(current, dt);
        recoverCVTRatio(current, dt);
        recoverMotorAssist(current, dt);
    }
    
    bool isRecoveryComplete(const MultiObjectiveOptimizer::ControlParameters& current) {
        return (std::abs(current.plow_depth_ratio - 1.0) < 0.02 &&
                std::abs(current.vehicle_speed_ratio - 1.0) < 0.02 &&
                current.motor_assist_ratio < 0.01);
    }
    
private:
    bool canStartRecovery(double current_load) {
        return current_load < 120.0; // 负载降到正常水平
    }
    
    void recoverDepth(MultiObjectiveOptimizer::ControlParameters& current, double dt) {
        if (current.plow_depth_ratio < 1.0) {
            double increment = recovery_rates_.depth_rate * dt / 0.25; // 归一化
            current.plow_depth_ratio = std::min(1.0, current.plow_depth_ratio + increment);
        }
    }
    
    void recoverSpeed(MultiObjectiveOptimizer::ControlParameters& current, double dt) {
        if (current.vehicle_speed_ratio < 1.0) {
            double increment = recovery_rates_.speed_rate * dt / 8.0; // 归一化到8km/h
            current.vehicle_speed_ratio = std::min(1.0, current.vehicle_speed_ratio + increment);
        }
    }
    
    void recoverCVTRatio(MultiObjectiveOptimizer::ControlParameters& current, double dt) {
        if (current.cvt_ratio < recovery_targets_.target_cvt_ratio) {
            current.cvt_ratio = std::min(recovery_targets_.target_cvt_ratio,
                                       current.cvt_ratio + recovery_rates_.cvt_rate * dt);
        }
    }
    
    void recoverMotorAssist(MultiObjectiveOptimizer::ControlParameters& current, double dt) {
        if (current.motor_assist_ratio > 0) {
            current.motor_assist_ratio = std::max(0.0,
                current.motor_assist_ratio - recovery_rates_.motor_rate * dt / 200.0);
        }
    }
};

// VCU智能决策控制器
class IntelligentVCUController {
private:
    VCUDecisionState current_state_;
    AnomalyDetector anomaly_detector_;
    MultiObjectiveOptimizer optimizer_;
    AdaptiveRecoveryController recovery_controller_;
    
    std::chrono::steady_clock::time_point anomaly_start_time_;
    MultiObjectiveOptimizer::ControlParameters current_params_;
    
    // 原始目标参数
    struct OriginalTargets {
        double plow_depth = 0.25;      // 25cm
        double vehicle_speed = 8.0;     // 8 km/h
        double cvt_ratio = 1.5;
        double engine_load = 0.65;      // 65%
    } original_targets_;

public:
    IntelligentVCUController() : current_state_(VCUDecisionState::NORMAL_OPERATION) {
        resetToNormalOperation();
    }
    
    void resetToNormalOperation() {
        current_params_.engine_torque_ratio = 0.65;
        current_params_.cvt_ratio = original_targets_.cvt_ratio;
        current_params_.plow_depth_ratio = 1.0;
        current_params_.vehicle_speed_ratio = 1.0;
        current_params_.motor_assist_ratio = 0.0;
    }
    
    void makeDecision(double current_load_torque, double dt, SystemState& state) {
        // 状态机决策逻辑
        switch (current_state_) {
            case VCUDecisionState::NORMAL_OPERATION:
                handleNormalOperation(current_load_torque, state);
                break;
                
            case VCUDecisionState::DETECTING_ANOMALY:
                handleAnomalyDetection(current_load_torque, dt, state);
                break;
                
            case VCUDecisionState::EMERGENCY_RESPONSE:
                handleEmergencyResponse(current_load_torque, dt, state);
                break;
                
            case VCUDecisionState::ADAPTIVE_ADJUSTMENT:
                handleAdaptiveAdjustment(current_load_torque, dt, state);
                break;
                
            case VCUDecisionState::RECOVERY_MODE:
                handleRecoveryMode(current_load_torque, dt, state);
                break;
                
            case VCUDecisionState::OPTIMIZATION_MODE:
                handleOptimizationMode(current_load_torque, dt, state);
                break;
        }
        
        // 应用决策结果
        applyDecisions(state);
        
        // 更新状态
        state.vcu_decision_state = static_cast<int>(current_state_);
    }
    
    VCUDecisionState getCurrentState() const { return current_state_; }
    
private:
    void handleNormalOperation(double load_torque, SystemState& state) {
        // 检测异常阻力
        if (anomaly_detector_.detectAnomaly(load_torque, state.time)) {
            std::cout << "[" << std::fixed << std::setprecision(1) << state.time 
                      << "s] VCU检测到异常阻力! 负载: " << load_torque << "Nm" << std::endl;
            
            current_state_ = VCUDecisionState::DETECTING_ANOMALY;
            anomaly_start_time_ = std::chrono::steady_clock::now();
            state.system_warnings = "异常阻力检测";
        }
    }
    
    void handleAnomalyDetection(double load_torque, double dt, SystemState& state) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - anomaly_start_time_).count();
        
        if (elapsed > 500) {  // 500ms确认期
            if (load_torque > 200.0) {  // 确认高阻力
                std::cout << "[" << std::fixed << std::setprecision(1) << state.time 
                          << "s] VCU确认高阻力情况，启动紧急响应" << std::endl;
                current_state_ = VCUDecisionState::EMERGENCY_RESPONSE;
                
                // 立即优化参数
                current_params_ = optimizer_.optimize(load_torque, 100.0);
                state.system_warnings = "紧急响应激活";
                
            } else {
                // 误报，返回正常状态
                current_state_ = VCUDecisionState::NORMAL_OPERATION;
                state.system_warnings = "";
            }
        }
    }
    
    void handleEmergencyResponse(double load_torque, double dt, SystemState& state) {
        // 如果阻力仍然很高，进一步调整
        if (load_torque > 250.0) {
            // 启用电机辅助
            if (current_params_.motor_assist_ratio < 0.01) {
                current_params_.motor_assist_ratio = 0.4;  // 40%辅助
                std::cout << "[" << std::fixed << std::setprecision(1) << state.time 
                          << "s] VCU启用混合动力模式" << std::endl;
                state.system_warnings = "混合动力激活";
            }
        }
        
        // 转入自适应调整模式
        current_state_ = VCUDecisionState::ADAPTIVE_ADJUSTMENT;
    }
    
    void handleAdaptiveAdjustment(double load_torque, double dt, SystemState& state) {
        // 根据当前阻力动态调整参数
        if (load_torque > 300.0) {
            // 极高阻力，进一步调整
            current_params_.plow_depth_ratio *= 0.98;
            current_params_.vehicle_speed_ratio *= 0.99;
            current_params_.motor_assist_ratio = std::min(0.6, current_params_.motor_assist_ratio + 0.01);
            
        } else if (load_torque < 180.0) {
            // 阻力开始减小，准备恢复
            std::cout << "[" << std::fixed << std::setprecision(1) << state.time 
                      << "s] VCU阻力减小，准备进入恢复模式" << std::endl;
            current_state_ = VCUDecisionState::RECOVERY_MODE;
            state.system_warnings = "准备恢复";
        }
        
        // 安全检查
        if (current_params_.plow_depth_ratio < 0.4) {
            state.system_warnings = "警告：犁地深度过小";
        }
    }
    
    void handleRecoveryMode(double load_torque, double dt, SystemState& state) {
        if (load_torque < 120.0) {  // 阻力恢复正常
            recovery_controller_.updateRecovery(current_params_, dt, load_torque);
            
            // 检查是否完全恢复
            if (recovery_controller_.isRecoveryComplete(current_params_)) {
                std::cout << "[" << std::fixed << std::setprecision(1) << state.time 
                          << "s] VCU完全恢复正常作业状态" << std::endl;
                current_state_ = VCUDecisionState::OPTIMIZATION_MODE;
                state.system_warnings = "恢复完成";
            } else {
                state.system_warnings = "参数恢复中";
            }
        }
    }
    
    void handleOptimizationMode(double load_torque, double dt, SystemState& state) {
        // 优化燃油效率和作业质量
        static double optimization_timer = 0.0;
        optimization_timer += dt;
        
        if (optimization_timer > 10.0) {  // 每10秒优化一次
            if (load_torque < 100.0) {
                current_params_.engine_torque_ratio *= 0.995;  // 轻微降低扭矩
            }
            optimization_timer = 0.0;
        }
        
        // 监控是否再次出现异常
        if (anomaly_detector_.detectAnomaly(load_torque, state.time)) {
            current_state_ = VCUDecisionState::DETECTING_ANOMALY;
            anomaly_start_time_ = std::chrono::steady_clock::now();
        }
        
        state.system_warnings = "优化模式";
    }
    
    void applyDecisions(SystemState& state) {
        // 应用发动机控制决策
        state.engine_throttle = current_params_.engine_torque_ratio;
        state.engine_torque = 400.0 * current_params_.engine_torque_ratio;  // 最大400Nm
        
        // 应用电机控制决策
        state.motor_torque = 200.0 * current_params_.motor_assist_ratio;  // 最大200Nm
        
        // 应用CVT控制决策
        state.cvt_ratio = current_params_.cvt_ratio;
        
        // 应用犁具深度控制决策
        state.plow_depth_target = 0.25 * current_params_.plow_depth_ratio;
        
        // 应用车速控制决策
        state.vehicle_speed = 8.0 * current_params_.vehicle_speed_ratio;
    }
};

// 土壤阻力模型
class SoilResistanceModel {
public:
    static double calculateResistanceFactor(double time) {
        if (time < 30.0) {
            // 阶段1: 正常土壤 (0-30s)
            return 1.0;
            
        } else if (time < 35.0) {
            // 阶段2: 阻力急剧上升 (30-35s)
            double t_norm = (time - 30.0) / 5.0;
            return 1.0 + 3.5 * (1.0 - exp(-5.0 * t_norm));
            
        } else if (time < 45.0) {
            // 阶段3: 高阻力持续 (35-45s)
            return 4.5 + 0.5 * sin(2.0 * M_PI * (time - 35.0));
            
        } else if (time < 60.0) {
            // 阶段4: 阻力逐渐减小 (45-60s)
            double t_norm = (time - 45.0) / 15.0;
            return 4.5 * exp(-2.0 * t_norm) + 1.0 * (1.0 - exp(-2.0 * t_norm));
            
        } else {
            // 阶段5: 恢复正常 (60s+)
            return 1.0;
        }
    }
};

// 犁具负载模型
class PlowLoadModel {
private:
    double working_width_ = 3.0;        // m
    double base_resistance_ = 15000.0;  // N/m²
    double effective_radius_ = 1.2;     // m
    
public:
    double calculateLoadTorque(double depth, double resistance_factor, double speed) {
        // 基础阻力计算
        double base_force = base_resistance_ * working_width_ * depth;
        
        // 深度影响（非线性）
        double depth_factor = 1.0 + 2.0 * std::pow(depth / 0.4, 2);
        
        // 速度影响
        double speed_factor = 1.0 + 0.1 * (speed / 8.0 - 1.0);
        
        // 总阻力
        double total_force = base_force * resistance_factor * depth_factor * speed_factor;
        
        // 转换为扭矩
        double load_torque = total_force * effective_radius_ / 1000.0;  // 转换为kNm然后到Nm
        
        return std::max(0.0, load_torque);
    }
};

// 动力总成模型
class PowertrainModel {
private:
    double engine_inertia_ = 0.5;       // kg⋅m²
    double motor_inertia_ = 0.2;        // kg⋅m²
    double battery_capacity_ = 100.0;   // kWh
    
public:
    void updatePowertrain(SystemState& state, double dt) {
        // 发动机模型
        updateEngine(state, dt);
        
        // 电机模型
        updateMotor(state, dt);
        
        // 电池模型
        updateBattery(state, dt);
        
        // 燃油消耗模型
        updateFuelConsumption(state, dt);
    }
    
private:
    void updateEngine(SystemState& state, double dt) {
        // 简化的发动机模型
        double target_speed = 1800.0 + 400.0 * state.engine_throttle;
        double speed_error = target_speed - state.engine_speed;
        state.engine_speed += speed_error * dt * 2.0;  // 一阶响应
        
        // 扭矩限制
        state.engine_torque = std::min(400.0, state.engine_torque);
    }
    
    void updateMotor(SystemState& state, double dt) {
        // 电机响应更快
        state.motor_speed = state.engine_speed * state.cvt_ratio;
        
        // 电机扭矩限制
        state.motor_torque = std::min(200.0, state.motor_torque);
    }
    
    void updateBattery(SystemState& state, double dt) {
        // 电池SOC计算
        double power_consumption = state.motor_torque * state.motor_speed * 2 * M_PI / 60.0 / 1000.0; // kW
        double soc_decrease = power_consumption * dt / 3600.0 / battery_capacity_ * 100.0; // %
        
        state.battery_soc = std::max(0.0, state.battery_soc - soc_decrease);
    }
    
    void updateFuelConsumption(SystemState& state, double dt) {
        // 燃油消耗模型
        double base_consumption = 15.0;  // L/h at idle
        double load_factor = state.engine_torque / 400.0;
        state.fuel_consumption_rate = base_consumption * (0.3 + 0.7 * load_factor);
    }
};

// 犁具深度控制模型
class PlowDepthController {
private:
    double hydraulic_time_constant_ = 0.5;  // s
    
public:
    void updateDepth(SystemState& state, double dt) {
        // 一阶滤波器模拟液压响应
        double depth_error = state.plow_depth_target - state.plow_depth_actual;
        state.plow_depth_actual += depth_error * dt / hydraulic_time_constant_;
        
        // 深度限制
        state.plow_depth_actual = std::clamp(state.plow_depth_actual, 0.05, 0.40);
    }
};

// 数据记录器
class DataLogger {
private:
    std::ofstream log_file_;
    
public:
    DataLogger(const std::string& filename) {
        log_file_.open(filename);
        writeHeader();
    }
    
    ~DataLogger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }
    
    void logData(const SystemState& state) {
        log_file_ << std::fixed << std::setprecision(3)
                  << state.time << ","
                  << state.soil_resistance_factor << ","
                  << state.engine_speed << ","
                  << state.engine_torque << ","
                  << state.engine_throttle << ","
                  << state.motor_torque << ","
                  << state.motor_speed << ","
                  << state.battery_soc << ","
                  << state.cvt_ratio << ","
                  << state.vehicle_speed << ","
                  << state.plow_depth_target << ","
                  << state.plow_depth_actual << ","
                  << state.plow_load_torque << ","
                  << state.fuel_consumption_rate << ","
                  << state.vcu_decision_state << ","
                  << "\"" << state.system_warnings << "\""
                  << std::endl;
    }
    
private:
    void writeHeader() {
        log_file_ << "Time,SoilResistanceFactor,EngineSpeed,EngineTorque,EngineThrottle,"
                  << "MotorTorque,MotorSpeed,BatterySOC,CVTRatio,VehicleSpeed,"
                  << "PlowDepthTarget,PlowDepthActual,PlowLoadTorque,FuelConsumptionRate,"
                  << "VCUDecisionState,SystemWarnings" << std::endl;
    }
};

// 性能监控器
class PerformanceMonitor {
private:
    struct TestMetrics {
        double anomaly_detection_time = -1.0;
        double max_engine_torque = 0.0;
        double min_plow_depth = 1.0;
        double max_motor_torque = 0.0;
        double recovery_start_time = -1.0;
        double recovery_complete_time = -1.0;
        bool hybrid_activated = false;
    } metrics_;
    
    double baseline_torque_ = 0.0;
    double baseline_depth_ = 0.0;
    bool anomaly_detected_ = false;
    
public:
    void updateMetrics(const SystemState& state) {
        // 记录基线值（前30秒）
        if (state.time < 30.0) {
            baseline_torque_ = state.engine_torque;
            baseline_depth_ = state.plow_depth_actual;
        }
        
        // 记录异常检测时间
        if (!anomaly_detected_ && state.vcu_decision_state == 1) {
            metrics_.anomaly_detection_time = state.time - 30.0;
            anomaly_detected_ = true;
            std::cout << "[监控] 异常检测时间: " << metrics_.anomaly_detection_time << "s" << std::endl;
        }
        
        // 记录最大发动机扭矩
        if (state.engine_torque > metrics_.max_engine_torque) {
            metrics_.max_engine_torque = state.engine_torque;
        }
        
        // 记录最小犁地深度
        if (state.plow_depth_actual < metrics_.min_plow_depth) {
            metrics_.min_plow_depth = state.plow_depth_actual;
        }
        
        // 记录最大电机扭矩
        if (state.motor_torque > metrics_.max_motor_torque) {
            metrics_.max_motor_torque = state.motor_torque;
        }
        
        // 记录混合动力激活
        if (state.motor_torque > 10.0 && !metrics_.hybrid_activated) {
            metrics_.hybrid_activated = true;
            std::cout << "[监控] 混合动力模式已激活" << std::endl;
        }
        
        // 记录恢复时间
        if (metrics_.recovery_start_time < 0 && state.vcu_decision_state == 4) {
            metrics_.recovery_start_time = state.time;
            std::cout << "[监控] 恢复模式开始: " << state.time << "s" << std::endl;
        }
        
        if (metrics_.recovery_complete_time < 0 && state.vcu_decision_state == 5) {
            metrics_.recovery_complete_time = state.time;
            std::cout << "[监控] 恢复完成: " << state.time << "s" << std::endl;
        }
    }
    
    bool validateResults() {
        bool passed = true;
        
        std::cout << "\n=== 测试结果验证 ===" << std::endl;
        
        // 验证异常检测时间
        if (metrics_.anomaly_detection_time > 2.0) {
            std::cout << "❌ 异常检测时间过长: " << metrics_.anomaly_detection_time << "s (要求≤2.0s)" << std::endl;
            passed = false;
        } else {
            std::cout << "✅ 异常检测时间: " << metrics_.anomaly_detection_time << "s" << std::endl;
        }
        
        // 验证发动机扭矩响应
        double torque_increase = (metrics_.max_engine_torque - baseline_torque_) / baseline_torque_;
        if (torque_increase < 0.20) {
            std::cout << "❌ 发动机扭矩增加不足: " << torque_increase * 100 << "% (要求≥20%)" << std::endl;
            passed = false;
        } else {
            std::cout << "✅ 发动机扭矩增加: " << torque_increase * 100 << "%" << std::endl;
        }
        
        // 验证犁地深度调整
        double depth_reduction = (baseline_depth_ - metrics_.min_plow_depth) / baseline_depth_;
        if (depth_reduction < 0.15) {
            std::cout << "❌ 犁地深度减少不足: " << depth_reduction * 100 << "% (要求≥15%)" << std::endl;
            passed = false;
        } else {
            std::cout << "✅ 犁地深度减少: " << depth_reduction * 100 << "%" << std::endl;
        }
        
        // 验证混合动力激活
        if (!metrics_.hybrid_activated) {
            std::cout << "❌ 混合动力模式未激活" << std::endl;
            passed = false;
        } else {
            std::cout << "✅ 混合动力模式已激活，最大扭矩: " << metrics_.max_motor_torque << "Nm" << std::endl;
        }
        
        // 验证恢复时间
        if (metrics_.recovery_complete_time > 0) {
            double recovery_duration = metrics_.recovery_complete_time - metrics_.recovery_start_time;
            if (recovery_duration > 30.0) {
                std::cout << "❌ 恢复时间过长: " << recovery_duration << "s (要求≤30s)" << std::endl;
                passed = false;
            } else {
                std::cout << "✅ 恢复时间: " << recovery_duration << "s" << std::endl;
            }
        }
        
        return passed;
    }
};

// 主仿真类
class PlowResistanceSimulation {
private:
    IntelligentVCUController vcu_controller_;
    SoilResistanceModel soil_model_;
    PlowLoadModel plow_load_model_;
    PowertrainModel powertrain_model_;
    PlowDepthController depth_controller_;
    DataLogger data_logger_;
    PerformanceMonitor performance_monitor_;
    
public:
    PlowResistanceSimulation() : data_logger_("plow_resistance_simulation_data.csv") {}
    
    bool runSimulation() {
        std::cout << "\n=== 开始犁地阻力智能决策仿真测试 ===" << std::endl;
        std::cout << "测试时长: 180秒" << std::endl;
        std::cout << "仿真步长: 10ms" << std::endl;
        std::cout << "数据记录: plow_resistance_simulation_data.csv" << std::endl;
        std::cout << "=" * 60 << std::endl;
        
        const double dt = 0.01;  // 10ms仿真步长
        const double total_duration = 180.0;  // 3分钟测试
        
        // 初始化系统状态
        SystemState state = initializeState();
        
        for (double time = 0.0; time < total_duration; time += dt) {
            state.time = time;
            
            // 更新土壤阻力
            state.soil_resistance_factor = soil_model_.calculateResistanceFactor(time);
            
            // 计算犁具负载
            state.plow_load_torque = plow_load_model_.calculateLoadTorque(
                state.plow_depth_actual, state.soil_resistance_factor, state.vehicle_speed);
            
            // VCU智能决策
            vcu_controller_.makeDecision(state.plow_load_torque, dt, state);
            
            // 更新物理模型
            powertrain_model_.updatePowertrain(state, dt);
            depth_controller_.updateDepth(state, dt);
            
            // 性能监控
            performance_monitor_.updateMetrics(state);
            
            // 记录数据
            data_logger_.logData(state);
            
            // 实时输出（每0.5秒）
            if (static_cast<int>(time * 100) % 50 == 0) {
                printProgress(state);
            }
        }
        
        std::cout << "\n" << "=" * 60 << std::endl;
        std::cout << "仿真完成！" << std::endl;
        
        // 验证测试结果
        bool passed = performance_monitor_.validateResults();
        
        std::cout << "\n总体结果: " << (passed ? "✅ 通过" : "❌ 失败") << std::endl;
        
        return passed;
    }
    
private:
    SystemState initializeState() {
        SystemState state = {};
        state.time = 0.0;
        state.soil_resistance_factor = 1.0;
        state.engine_speed = 1800.0;
        state.engine_torque = 260.0;
        state.engine_throttle = 0.65;
        state.motor_torque = 0.0;
        state.motor_speed = 0.0;
        state.battery_soc = 85.0;
        state.cvt_ratio = 1.5;
        state.vehicle_speed = 8.0;
        state.plow_depth_target = 0.25;
        state.plow_depth_actual = 0.25;
        state.plow_load_torque = 85.0;
        state.fuel_consumption_rate = 18.0;
        state.vcu_decision_state = 0;
        state.system_warnings = "";
        
        return state;
    }
    
    void printProgress(const SystemState& state) {
        std::cout << std::fixed << std::setprecision(1)
                  << "[" << state.time << "s] "
                  << "负载:" << state.plow_load_torque << "Nm "
                  << "阻力:" << state.soil_resistance_factor << " "
                  << "状态:" << state.vcu_decision_state << " "
                  << "深度:" << state.plow_depth_actual << "m "
                  << "速度:" << state.vehicle_speed << "km/h";
        
        if (!state.system_warnings.empty()) {
            std::cout << " [" << state.system_warnings << "]";
        }
        
        std::cout << std::endl;
    }
};

// 主函数
int main() {
    std::cout << "VCU犁地阻力智能决策仿真测试程序" << std::endl;
    std::cout << "版本: 1.0" << std::endl;
    std::cout << "作者: VCU开发团队" << std::endl;
    
    PlowResistanceSimulation simulation;
    
    bool result = simulation.runSimulation();
    
    std::cout << "\n推荐后续操作:" << std::endl;
    std::cout << "1. 运行数据分析: python3 analyze_test_results.py" << std::endl;
    std::cout << "2. 查看数据文件: plow_resistance_simulation_data.csv" << std::endl;
    std::cout << "3. 生成可视化图表和报告" << std::endl;
    
    return result ? 0 : 1;
}
