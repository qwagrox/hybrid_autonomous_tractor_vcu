// enhanced_plow_model.hpp
// 增强的犁具模型，支持动态阻力变化和智能决策测试

#ifndef ENHANCED_PLOW_MODEL_HPP
#define ENHANCED_PLOW_MODEL_HPP

#include <cmath>
#include <algorithm>
#include <iostream>

class EnhancedPlowModel {
private:
    // 犁具物理参数
    double working_width_;          // 工作宽度 (m)
    double current_depth_;          // 当前犁地深度 (m)
    double target_depth_;           // 目标犁地深度 (m)
    double max_depth_;              // 最大犁地深度 (m)
    double min_depth_;              // 最小犁地深度 (m)
    
    // 土壤和环境参数
    double base_soil_resistance_;   // 基础土壤阻力系数
    double resistance_factor_;      // 动态阻力系数 (1.0 = 正常, >1.0 = 高阻力)
    double soil_moisture_;          // 土壤湿度 (0-1)
    double soil_density_;           // 土壤密度 (kg/m³)
    
    // 液压系统参数
    double hydraulic_response_time_; // 液压响应时间常数 (s)
    double max_hydraulic_force_;     // 最大液压力 (N)
    double current_hydraulic_force_; // 当前液压力 (N)
    
    // 动力学参数
    double load_torque_;            // 当前负载扭矩 (Nm)
    double draft_force_;            // 牵引阻力 (N)
    double vertical_force_;         // 垂直力 (N)
    double side_force_;             // 侧向力 (N)
    
    // 磨损和效率参数
    double wear_factor_;            // 磨损系数 (0-1, 1=全新)
    double efficiency_;             // 犁地效率 (0-1)
    
    // 故障模拟参数
    bool hydraulic_fault_;          // 液压系统故障
    bool depth_sensor_fault_;       // 深度传感器故障
    double sensor_noise_amplitude_; // 传感器噪声幅度

public:
    EnhancedPlowModel(double working_width = 3.0) :
        working_width_(working_width),
        current_depth_(0.0),
        target_depth_(0.25),
        max_depth_(0.40),
        min_depth_(0.05),
        base_soil_resistance_(15000.0),  // N/m²
        resistance_factor_(1.0),
        soil_moisture_(0.3),
        soil_density_(1400.0),
        hydraulic_response_time_(0.5),
        max_hydraulic_force_(50000.0),
        current_hydraulic_force_(0.0),
        load_torque_(0.0),
        draft_force_(0.0),
        vertical_force_(0.0),
        side_force_(0.0),
        wear_factor_(1.0),
        efficiency_(0.95),
        hydraulic_fault_(false),
        depth_sensor_fault_(false),
        sensor_noise_amplitude_(0.0) {}
    
    void update(double dt) {
        // 更新液压系统响应
        updateHydraulicSystem(dt);
        
        // 计算土壤阻力
        calculateSoilResistance();
        
        // 计算各种力
        calculateForces();
        
        // 计算负载扭矩
        calculateLoadTorque();
        
        // 更新磨损和效率
        updateWearAndEfficiency(dt);
        
        // 模拟故障影响
        applyFaultEffects();
    }
    
    // 设置控制指令
    void setCommand(const std::string& command, double value) {
        if (command == "depth") {
            target_depth_ = std::clamp(value, min_depth_, max_depth_);
        } else if (command == "resistance_factor") {
            resistance_factor_ = std::max(0.1, value);
        } else if (command == "soil_moisture") {
            soil_moisture_ = std::clamp(value, 0.0, 1.0);
        } else if (command == "hydraulic_force") {
            current_hydraulic_force_ = std::clamp(value, 0.0, max_hydraulic_force_);
        }
    }
    
    // 获取状态信息
    double getCurrentDepth() const { 
        if (depth_sensor_fault_) {
            // 模拟传感器故障
            return current_depth_ + sensor_noise_amplitude_ * (rand() / double(RAND_MAX) - 0.5);
        }
        return current_depth_; 
    }
    
    double getTargetDepth() const { return target_depth_; }
    double getLoadTorque() const { return load_torque_; }
    double getDraftForce() const { return draft_force_; }
    double getVerticalForce() const { return vertical_force_; }
    double getEfficiency() const { return efficiency_; }
    double getResistanceFactor() const { return resistance_factor_; }
    
    // 获取详细状态
    struct PlowStatus {
        double current_depth;
        double target_depth;
        double load_torque;
        double draft_force;
        double vertical_force;
        double side_force;
        double hydraulic_force;
        double resistance_factor;
        double efficiency;
        double wear_factor;
        bool hydraulic_fault;
        bool depth_sensor_fault;
        std::string operation_mode;
    };
    
    PlowStatus getDetailedStatus() const {
        PlowStatus status;
        status.current_depth = getCurrentDepth();
        status.target_depth = target_depth_;
        status.load_torque = load_torque_;
        status.draft_force = draft_force_;
        status.vertical_force = vertical_force_;
        status.side_force = side_force_;
        status.hydraulic_force = current_hydraulic_force_;
        status.resistance_factor = resistance_factor_;
        status.efficiency = efficiency_;
        status.wear_factor = wear_factor_;
        status.hydraulic_fault = hydraulic_fault_;
        status.depth_sensor_fault = depth_sensor_fault_;
        
        // 确定操作模式
        if (resistance_factor_ > 3.0) {
            status.operation_mode = "HIGH_RESISTANCE";
        } else if (resistance_factor_ > 1.5) {
            status.operation_mode = "MODERATE_RESISTANCE";
        } else if (std::abs(current_depth_ - target_depth_) > 0.02) {
            status.operation_mode = "ADJUSTING_DEPTH";
        } else {
            status.operation_mode = "NORMAL_OPERATION";
        }
        
        return status;
    }
    
    // 故障注入接口
    void injectFault(const std::string& fault_type, bool enable, double parameter = 0.0) {
        if (fault_type == "hydraulic_fault") {
            hydraulic_fault_ = enable;
        } else if (fault_type == "depth_sensor_fault") {
            depth_sensor_fault_ = enable;
            sensor_noise_amplitude_ = parameter;
        }
    }
    
    // 重置到初始状态
    void reset() {
        current_depth_ = 0.0;
        target_depth_ = 0.25;
        resistance_factor_ = 1.0;
        current_hydraulic_force_ = 0.0;
        load_torque_ = 0.0;
        wear_factor_ = 1.0;
        efficiency_ = 0.95;
        hydraulic_fault_ = false;
        depth_sensor_fault_ = false;
        sensor_noise_amplitude_ = 0.0;
    }

private:
    void updateHydraulicSystem(double dt) {
        if (hydraulic_fault_) {
            // 液压故障时响应变慢
            hydraulic_response_time_ = 2.0;
            max_hydraulic_force_ *= 0.6;  // 力量减少40%
        }
        
        // 一阶滤波器模拟液压响应
        double depth_error = target_depth_ - current_depth_;
        double required_force = depth_error * 20000.0;  // 比例控制
        required_force = std::clamp(required_force, -max_hydraulic_force_, max_hydraulic_force_);
        
        // 液压力响应
        double force_error = required_force - current_hydraulic_force_;
        current_hydraulic_force_ += force_error * dt / hydraulic_response_time_;
        
        // 深度变化（简化模型）
        double depth_change_rate = current_hydraulic_force_ / 25000.0;  // m/s per N
        current_depth_ += depth_change_rate * dt;
        current_depth_ = std::clamp(current_depth_, 0.0, max_depth_);
    }
    
    void calculateSoilResistance() {
        // 基础阻力计算
        double base_resistance = base_soil_resistance_ * working_width_ * current_depth_;
        
        // 土壤湿度影响
        double moisture_factor = 1.0 + 0.5 * soil_moisture_;  // 湿度增加阻力
        
        // 深度影响（非线性）
        double depth_factor = 1.0 + 2.0 * std::pow(current_depth_ / max_depth_, 2);
        
        // 速度影响（假设恒定速度）
        double speed_factor = 1.0;  // 可以根据车速调整
        
        // 总阻力
        double total_resistance = base_resistance * resistance_factor_ * 
                                moisture_factor * depth_factor * speed_factor * wear_factor_;
        
        draft_force_ = total_resistance;
    }
    
    void calculateForces() {
        // 牵引阻力已在 calculateSoilResistance() 中计算
        
        // 垂直力（犁具重量 + 土壤反力）
        double plow_weight = 2000.0;  // N
        double soil_reaction = current_depth_ * working_width_ * soil_density_ * 9.81 * 0.3;
        vertical_force_ = plow_weight + soil_reaction;
        
        // 侧向力（由于土壤不均匀性）
        side_force_ = draft_force_ * 0.1 * resistance_factor_;  // 阻力越大侧向力越大
    }
    
    void calculateLoadTorque() {
        // 假设有效半径为1.2m（从PTO到犁具的等效半径）
        double effective_radius = 1.2;
        
        // 基础负载扭矩
        load_torque_ = draft_force_ * effective_radius / 1000.0;  // 转换为kNm然后到Nm
        
        // 添加动态成分
        double dynamic_component = 0.0;
        if (resistance_factor_ > 2.0) {
            // 高阻力时的冲击负载
            dynamic_component = load_torque_ * 0.2 * sin(10.0 * resistance_factor_);
        }
        
        load_torque_ += dynamic_component;
        
        // 考虑效率损失
        load_torque_ /= efficiency_;
        
        // 确保非负
        load_torque_ = std::max(0.0, load_torque_);
    }
    
    void updateWearAndEfficiency(double dt) {
        // 磨损随时间和阻力增加
        double wear_rate = 0.00001 * resistance_factor_ * dt;  // 每秒磨损率
        wear_factor_ = std::max(0.5, wear_factor_ - wear_rate);
        
        // 效率随磨损降低
        efficiency_ = 0.95 * wear_factor_;
        
        // 高阻力时效率临时降低
        if (resistance_factor_ > 2.0) {
            efficiency_ *= (1.0 - 0.1 * (resistance_factor_ - 2.0) / 3.0);
        }
    }
    
    void applyFaultEffects() {
        if (hydraulic_fault_) {
            // 液压故障影响深度控制精度
            double fault_noise = 0.01 * (rand() / double(RAND_MAX) - 0.5);
            current_depth_ += fault_noise;
            current_depth_ = std::clamp(current_depth_, 0.0, max_depth_);
        }
    }

public:
    // 诊断接口
    std::string getDiagnosticInfo() const {
        std::string info = "犁具诊断信息:\n";
        info += "  当前深度: " + std::to_string(current_depth_) + " m\n";
        info += "  目标深度: " + std::to_string(target_depth_) + " m\n";
        info += "  阻力系数: " + std::to_string(resistance_factor_) + "\n";
        info += "  负载扭矩: " + std::to_string(load_torque_) + " Nm\n";
        info += "  牵引力: " + std::to_string(draft_force_) + " N\n";
        info += "  液压力: " + std::to_string(current_hydraulic_force_) + " N\n";
        info += "  效率: " + std::to_string(efficiency_ * 100) + " %\n";
        info += "  磨损: " + std::to_string((1.0 - wear_factor_) * 100) + " %\n";
        
        if (hydraulic_fault_) info += "  ⚠️ 液压系统故障\n";
        if (depth_sensor_fault_) info += "  ⚠️ 深度传感器故障\n";
        
        return info;
    }
    
    // 性能评估
    struct PerformanceMetrics {
        double average_depth_error;
        double max_load_torque;
        double efficiency_rating;
        double stability_index;
        int fault_count;
    };
    
    PerformanceMetrics evaluatePerformance(const std::vector<double>& depth_history,
                                         const std::vector<double>& torque_history) const {
        PerformanceMetrics metrics;
        
        // 计算平均深度误差
        double total_error = 0.0;
        for (double depth : depth_history) {
            total_error += std::abs(depth - target_depth_);
        }
        metrics.average_depth_error = total_error / depth_history.size();
        
        // 最大负载扭矩
        metrics.max_load_torque = *std::max_element(torque_history.begin(), torque_history.end());
        
        // 效率评级
        metrics.efficiency_rating = efficiency_;
        
        // 稳定性指数（基于扭矩变化）
        double torque_variance = 0.0;
        double mean_torque = 0.0;
        for (double torque : torque_history) {
            mean_torque += torque;
        }
        mean_torque /= torque_history.size();
        
        for (double torque : torque_history) {
            torque_variance += std::pow(torque - mean_torque, 2);
        }
        torque_variance /= torque_history.size();
        
        metrics.stability_index = 1.0 / (1.0 + torque_variance / 1000.0);  // 归一化稳定性指数
        
        // 故障计数
        metrics.fault_count = (hydraulic_fault_ ? 1 : 0) + (depth_sensor_fault_ ? 1 : 0);
        
        return metrics;
    }
};

#endif // ENHANCED_PLOW_MODEL_HPP
