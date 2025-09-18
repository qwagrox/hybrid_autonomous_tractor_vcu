// intelligent_decision_test.cpp
// VCU智能决策测试 - 犁地突发阻力场景

#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>
#include "models/powertrain_model.hpp"
#include "models/implement_model.hpp"
#include "models/environment_model.hpp"

// VCU决策状态枚举
enum class VCUDecisionState {
    NORMAL_OPERATION,
    DETECTING_ANOMALY,
    EMERGENCY_RESPONSE,
    ADAPTIVE_ADJUSTMENT,
    RECOVERY_MODE,
    OPTIMIZATION_MODE
};

// VCU智能决策控制器
class IntelligentVCUController {
private:
    VCUDecisionState current_state_;
    double resistance_detection_threshold_;
    double last_load_torque_;
    double load_torque_rate_;
    std::chrono::steady_clock::time_point anomaly_start_time_;
    
    // 决策参数
    struct DecisionParameters {
        double target_engine_torque;
        double target_cvt_ratio;
        double target_plow_depth;
        double target_vehicle_speed;
        double motor_assist_torque;
        bool hybrid_mode_active;
    } decision_params_;
    
    // 原始目标参数（用于恢复）
    struct OriginalTargets {
        double plow_depth = 0.25;      // 25cm
        double vehicle_speed = 8.0;     // 8 km/h
        double cvt_ratio = 1.5;
        double engine_load = 60.0;      // 60%
    } original_targets_;

public:
    IntelligentVCUController() : 
        current_state_(VCUDecisionState::NORMAL_OPERATION),
        resistance_detection_threshold_(1.5),  // 阻力增加50%触发检测
        last_load_torque_(0.0),
        load_torque_rate_(0.0) {
        
        // 初始化决策参数为正常值
        resetToNormalOperation();
    }
    
    void resetToNormalOperation() {
        decision_params_.target_engine_torque = 240.0;  // Nm
        decision_params_.target_cvt_ratio = original_targets_.cvt_ratio;
        decision_params_.target_plow_depth = original_targets_.plow_depth;
        decision_params_.target_vehicle_speed = original_targets_.vehicle_speed;
        decision_params_.motor_assist_torque = 0.0;
        decision_params_.hybrid_mode_active = false;
    }
    
    // 主决策函数
    void makeDecision(double current_load_torque, double dt, 
                     PowertrainModel& powertrain, PlowModel& plow) {
        
        // 计算负载变化率
        load_torque_rate_ = (current_load_torque - last_load_torque_) / dt;
        last_load_torque_ = current_load_torque;
        
        // 状态机决策逻辑
        switch (current_state_) {
            case VCUDecisionState::NORMAL_OPERATION:
                handleNormalOperation(current_load_torque);
                break;
                
            case VCUDecisionState::DETECTING_ANOMALY:
                handleAnomalyDetection(current_load_torque, dt);
                break;
                
            case VCUDecisionState::EMERGENCY_RESPONSE:
                handleEmergencyResponse(current_load_torque, dt);
                break;
                
            case VCUDecisionState::ADAPTIVE_ADJUSTMENT:
                handleAdaptiveAdjustment(current_load_torque, dt);
                break;
                
            case VCUDecisionState::RECOVERY_MODE:
                handleRecoveryMode(current_load_torque, dt);
                break;
                
            case VCUDecisionState::OPTIMIZATION_MODE:
                handleOptimizationMode(current_load_torque, dt);
                break;
        }
        
        // 应用决策结果
        applyDecisions(powertrain, plow);
        
        // 记录决策日志
        logDecision(current_load_torque);
    }
    
private:
    void handleNormalOperation(double load_torque) {
        // 检测异常阻力
        if (load_torque > 150.0 && load_torque_rate_ > 50.0) {  // 负载>150Nm且变化率>50Nm/s
            std::cout << "[VCU] 检测到异常阻力! 负载: " << load_torque 
                      << "Nm, 变化率: " << load_torque_rate_ << "Nm/s" << std::endl;
            
            current_state_ = VCUDecisionState::DETECTING_ANOMALY;
            anomaly_start_time_ = std::chrono::steady_clock::now();
        }
    }
    
    void handleAnomalyDetection(double load_torque, double dt) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - anomaly_start_time_).count();
        
        if (elapsed > 500) {  // 500ms确认期
            if (load_torque > 200.0) {  // 确认高阻力
                std::cout << "[VCU] 确认高阻力情况，启动紧急响应" << std::endl;
                current_state_ = VCUDecisionState::EMERGENCY_RESPONSE;
                
                // 立即增加发动机扭矩
                decision_params_.target_engine_torque = std::min(380.0, 
                    decision_params_.target_engine_torque * 1.3);
                
                // 降低CVT传动比以增加扭矩
                decision_params_.target_cvt_ratio = std::max(0.8, 
                    decision_params_.target_cvt_ratio * 0.7);
                    
            } else {
                // 误报，返回正常状态
                current_state_ = VCUDecisionState::NORMAL_OPERATION;
            }
        }
    }
    
    void handleEmergencyResponse(double load_torque, double dt) {
        std::cout << "[VCU] 紧急响应模式 - 调整动力系统参数" << std::endl;
        
        // 如果阻力仍然很高，进一步调整
        if (load_torque > 250.0) {
            // 减小犁地深度
            decision_params_.target_plow_depth = std::max(0.10, 
                decision_params_.target_plow_depth * 0.85);
            
            // 降低车速
            decision_params_.target_vehicle_speed = std::max(4.0,
                decision_params_.target_vehicle_speed * 0.8);
            
            // 启用电机辅助
            if (!decision_params_.hybrid_mode_active) {
                decision_params_.hybrid_mode_active = true;
                decision_params_.motor_assist_torque = 80.0;  // 80Nm辅助扭矩
                std::cout << "[VCU] 启用混合动力模式，电机辅助扭矩: 80Nm" << std::endl;
            }
        }
        
        // 转入自适应调整模式
        current_state_ = VCUDecisionState::ADAPTIVE_ADJUSTMENT;
    }
    
    void handleAdaptiveAdjustment(double load_torque, double dt) {
        // 根据当前阻力动态调整参数
        if (load_torque > 300.0) {
            // 极高阻力，进一步减小深度和速度
            decision_params_.target_plow_depth *= 0.95;
            decision_params_.target_vehicle_speed *= 0.98;
            decision_params_.motor_assist_torque = std::min(120.0, 
                decision_params_.motor_assist_torque + 5.0);
                
        } else if (load_torque < 180.0) {
            // 阻力开始减小，准备恢复
            std::cout << "[VCU] 阻力减小，准备进入恢复模式" << std::endl;
            current_state_ = VCUDecisionState::RECOVERY_MODE;
        }
        
        // 安全检查
        if (decision_params_.target_plow_depth < 0.08) {
            std::cout << "[VCU] 警告：犁地深度过小，建议停止作业检查" << std::endl;
        }
    }
    
    void handleRecoveryMode(double load_torque, double dt) {
        std::cout << "[VCU] 恢复模式 - 逐步恢复作业参数" << std::endl;
        
        if (load_torque < 120.0) {  // 阻力恢复正常
            // 逐步恢复犁地深度
            if (decision_params_.target_plow_depth < original_targets_.plow_depth) {
                decision_params_.target_plow_depth = std::min(original_targets_.plow_depth,
                    decision_params_.target_plow_depth + 0.01 * dt);  // 每秒恢复1cm
            }
            
            // 逐步恢复车速
            if (decision_params_.target_vehicle_speed < original_targets_.vehicle_speed) {
                decision_params_.target_vehicle_speed = std::min(original_targets_.vehicle_speed,
                    decision_params_.target_vehicle_speed + 0.5 * dt);  // 每秒恢复0.5km/h
            }
            
            // 逐步恢复CVT传动比
            if (decision_params_.target_cvt_ratio < original_targets_.cvt_ratio) {
                decision_params_.target_cvt_ratio = std::min(original_targets_.cvt_ratio,
                    decision_params_.target_cvt_ratio + 0.1 * dt);
            }
            
            // 逐步减少电机辅助
            if (decision_params_.motor_assist_torque > 0) {
                decision_params_.motor_assist_torque = std::max(0.0,
                    decision_params_.motor_assist_torque - 10.0 * dt);  // 每秒减少10Nm
                
                if (decision_params_.motor_assist_torque <= 5.0) {
                    decision_params_.hybrid_mode_active = false;
                    decision_params_.motor_assist_torque = 0.0;
                }
            }
            
            // 检查是否完全恢复
            if (std::abs(decision_params_.target_plow_depth - original_targets_.plow_depth) < 0.01 &&
                std::abs(decision_params_.target_vehicle_speed - original_targets_.vehicle_speed) < 0.2 &&
                !decision_params_.hybrid_mode_active) {
                
                std::cout << "[VCU] 完全恢复正常作业状态" << std::endl;
                current_state_ = VCUDecisionState::OPTIMIZATION_MODE;
            }
        }
    }
    
    void handleOptimizationMode(double load_torque, double dt) {
        // 优化燃油效率和作业质量
        static double optimization_timer = 0.0;
        optimization_timer += dt;
        
        if (optimization_timer > 10.0) {  // 每10秒优化一次
            // 微调发动机负载以优化燃油效率
            if (load_torque < 100.0) {
                decision_params_.target_engine_torque *= 0.98;  // 轻微降低扭矩
            }
            
            optimization_timer = 0.0;
        }
        
        // 监控是否再次出现异常
        if (load_torque > 150.0 && load_torque_rate_ > 30.0) {
            current_state_ = VCUDecisionState::DETECTING_ANOMALY;
            anomaly_start_time_ = std::chrono::steady_clock::now();
        }
    }
    
    void applyDecisions(PowertrainModel& powertrain, PlowModel& plow) {
        // 应用发动机控制决策
        double throttle = decision_params_.target_engine_torque / 400.0;  // 转换为油门开度
        
        // 应用电机控制决策
        double motor_command = decision_params_.motor_assist_torque / 200.0;
        
        // 应用CVT控制决策
        // (在实际系统中会发送CAN消息给CVT控制器)
        
        // 应用犁具深度控制决策
        plow.setCommand("depth", decision_params_.target_plow_depth);
        
        // 更新动力总成模型
        double total_load = plow.getLoadTorque();
        powertrain.update(throttle, motor_command, decision_params_.target_cvt_ratio, 
                         total_load, 0.01);
    }
    
    void logDecision(double load_torque) {
        static std::ofstream log_file("vcu_decision_log.csv", std::ios::app);
        static bool header_written = false;
        
        if (!header_written) {
            log_file << "Time,State,LoadTorque,EngTorque,CVTRatio,PlowDepth,VehicleSpeed,MotorTorque,HybridMode\n";
            header_written = true;
        }
        
        log_file << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count() << ","
                 << static_cast<int>(current_state_) << ","
                 << load_torque << ","
                 << decision_params_.target_engine_torque << ","
                 << decision_params_.target_cvt_ratio << ","
                 << decision_params_.target_plow_depth << ","
                 << decision_params_.target_vehicle_speed << ","
                 << decision_params_.motor_assist_torque << ","
                 << (decision_params_.hybrid_mode_active ? 1 : 0) << "\n";
    }

public:
    // 获取当前决策状态（用于测试验证）
    VCUDecisionState getCurrentState() const { return current_state_; }
    
    // 获取决策参数（用于测试验证）
    const DecisionParameters& getDecisionParameters() const { return decision_params_; }
    
    // 强制状态转换（用于测试）
    void forceState(VCUDecisionState state) { current_state_ = state; }
};

// 智能决策测试主程序
class IntelligentDecisionTestRunner {
private:
    PowertrainModel powertrain_;
    PlowModel plow_;
    EnvironmentModel environment_;
    IntelligentVCUController vcu_controller_;
    
    // 测试结果记录
    struct TestResults {
        double anomaly_detection_time = -1.0;
        double max_engine_torque = 0.0;
        double min_plow_depth = 1.0;
        double recovery_completion_time = -1.0;
        bool hybrid_mode_activated = false;
        std::vector<std::string> decision_sequence;
    } test_results_;

public:
    bool runCriticalPlowResistanceTest() {
        std::cout << "\n=== 开始VCU智能决策测试：犁地突发阻力场景 ===" << std::endl;
        
        const double dt = 0.01;  // 10ms仿真步长
        const double total_duration = 180.0;  // 3分钟测试
        
        bool test_passed = true;
        double anomaly_start_time = 30.0;  // 30秒时开始异常
        double anomaly_end_time = 60.0;    // 60秒时异常结束
        
        for (double time = 0.0; time < total_duration; time += dt) {
            // 更新环境阻力
            double resistance_factor = calculateResistanceFactor(time, anomaly_start_time, anomaly_end_time);
            
            // 更新犁具模型（包含阻力变化）
            plow_.setCommand("resistance_factor", resistance_factor);
            plow_.update(dt);
            
            // VCU智能决策
            double current_load = plow_.getLoadTorque();
            vcu_controller_.makeDecision(current_load, dt, powertrain_, plow_);
            
            // 记录关键事件
            recordKeyEvents(time, current_load, anomaly_start_time);
            
            // 实时输出关键信息
            if (static_cast<int>(time * 10) % 50 == 0) {  // 每0.5秒输出一次
                printStatus(time, current_load, resistance_factor);
            }
        }
        
        // 分析测试结果
        test_passed = analyzeTestResults();
        
        // 生成测试报告
        generateTestReport(test_passed);
        
        return test_passed;
    }

private:
    double calculateResistanceFactor(double time, double start_time, double end_time) {
        if (time < start_time) {
            return 1.0;  // 正常阻力
        } else if (time < start_time + 15.0) {
            // 30-45秒：阻力剧增
            return 1.0 + 3.5 * (time - start_time) / 15.0;  // 从1.0增加到4.5
        } else if (time < end_time) {
            // 45-60秒：阻力逐渐减小
            return 4.5 - 1.7 * (time - start_time - 15.0) / 15.0;  // 从4.5减少到2.8
        } else {
            // 60秒后：恢复正常
            double recovery_time = time - end_time;
            if (recovery_time < 10.0) {
                return 2.8 - 1.8 * recovery_time / 10.0;  // 10秒内从2.8恢复到1.0
            } else {
                return 1.0;
            }
        }
    }
    
    void recordKeyEvents(double time, double load_torque, double anomaly_start_time) {
        auto current_state = vcu_controller_.getCurrentState();
        auto params = vcu_controller_.getDecisionParameters();
        
        // 记录异常检测时间
        if (test_results_.anomaly_detection_time < 0 && 
            current_state == VCUDecisionState::DETECTING_ANOMALY) {
            test_results_.anomaly_detection_time = time - anomaly_start_time;
            std::cout << "[测试] 异常检测时间: " << test_results_.anomaly_detection_time << "秒" << std::endl;
        }
        
        // 记录最大发动机扭矩
        if (params.target_engine_torque > test_results_.max_engine_torque) {
            test_results_.max_engine_torque = params.target_engine_torque;
        }
        
        // 记录最小犁地深度
        if (params.target_plow_depth < test_results_.min_plow_depth) {
            test_results_.min_plow_depth = params.target_plow_depth;
        }
        
        // 记录混合动力模式激活
        if (params.hybrid_mode_active && !test_results_.hybrid_mode_activated) {
            test_results_.hybrid_mode_activated = true;
            std::cout << "[测试] 混合动力模式已激活" << std::endl;
        }
        
        // 记录恢复完成时间
        if (test_results_.recovery_completion_time < 0 && 
            current_state == VCUDecisionState::OPTIMIZATION_MODE) {
            test_results_.recovery_completion_time = time;
            std::cout << "[测试] 系统恢复完成时间: " << time << "秒" << std::endl;
        }
    }
    
    void printStatus(double time, double load_torque, double resistance_factor) {
        auto state = vcu_controller_.getCurrentState();
        auto params = vcu_controller_.getDecisionParameters();
        
        std::cout << std::fixed << std::setprecision(1)
                  << "[" << time << "s] "
                  << "负载:" << load_torque << "Nm "
                  << "阻力系数:" << resistance_factor << " "
                  << "状态:" << static_cast<int>(state) << " "
                  << "深度:" << params.target_plow_depth << "m "
                  << "速度:" << params.target_vehicle_speed << "km/h"
                  << std::endl;
    }
    
    bool analyzeTestResults() {
        bool passed = true;
        
        std::cout << "\n=== 测试结果分析 ===" << std::endl;
        
        // 检查异常检测时间
        if (test_results_.anomaly_detection_time > 2.0) {
            std::cout << "❌ 异常检测时间过长: " << test_results_.anomaly_detection_time << "s (要求<2s)" << std::endl;
            passed = false;
        } else {
            std::cout << "✅ 异常检测时间: " << test_results_.anomaly_detection_time << "s" << std::endl;
        }
        
        // 检查发动机扭矩响应
        if (test_results_.max_engine_torque < 288.0) {  // 240 * 1.2 = 288
            std::cout << "❌ 发动机扭矩增加不足: " << test_results_.max_engine_torque << "Nm (要求>288Nm)" << std::endl;
            passed = false;
        } else {
            std::cout << "✅ 最大发动机扭矩: " << test_results_.max_engine_torque << "Nm" << std::endl;
        }
        
        // 检查犁地深度调整
        double depth_reduction = (0.25 - test_results_.min_plow_depth) / 0.25 * 100;
        if (depth_reduction < 15.0) {
            std::cout << "❌ 犁地深度减少不足: " << depth_reduction << "% (要求>15%)" << std::endl;
            passed = false;
        } else {
            std::cout << "✅ 犁地深度减少: " << depth_reduction << "%" << std::endl;
        }
        
        // 检查混合动力模式
        if (!test_results_.hybrid_mode_activated) {
            std::cout << "❌ 混合动力模式未激活" << std::endl;
            passed = false;
        } else {
            std::cout << "✅ 混合动力模式已激活" << std::endl;
        }
        
        // 检查恢复时间
        if (test_results_.recovery_completion_time < 0 || 
            test_results_.recovery_completion_time > 90.0) {
            std::cout << "❌ 系统恢复时间异常" << std::endl;
            passed = false;
        } else {
            std::cout << "✅ 系统恢复时间: " << test_results_.recovery_completion_time << "s" << std::endl;
        }
        
        return passed;
    }
    
    void generateTestReport(bool passed) {
        std::ofstream report("intelligent_decision_test_report.html");
        
        report << "<!DOCTYPE html><html><head><title>VCU智能决策测试报告</title></head><body>";
        report << "<h1>VCU智能决策测试报告 - 犁地突发阻力场景</h1>";
        report << "<h2>测试结果: " << (passed ? "✅ 通过" : "❌ 失败") << "</h2>";
        
        report << "<h3>关键指标</h3><table border='1'>";
        report << "<tr><th>指标</th><th>实际值</th><th>要求</th><th>状态</th></tr>";
        report << "<tr><td>异常检测时间</td><td>" << test_results_.anomaly_detection_time 
               << "s</td><td>&lt;2s</td><td>" << (test_results_.anomaly_detection_time <= 2.0 ? "✅" : "❌") << "</td></tr>";
        report << "<tr><td>最大发动机扭矩</td><td>" << test_results_.max_engine_torque 
               << "Nm</td><td>&gt;288Nm</td><td>" << (test_results_.max_engine_torque >= 288.0 ? "✅" : "❌") << "</td></tr>";
        report << "<tr><td>最小犁地深度</td><td>" << test_results_.min_plow_depth 
               << "m</td><td>&lt;0.21m</td><td>" << (test_results_.min_plow_depth <= 0.21 ? "✅" : "❌") << "</td></tr>";
        report << "<tr><td>混合动力激活</td><td>" << (test_results_.hybrid_mode_activated ? "是" : "否")
               << "</td><td>是</td><td>" << (test_results_.hybrid_mode_activated ? "✅" : "❌") << "</td></tr>";
        report << "</table>";
        
        report << "<p>详细测试数据请查看 vcu_decision_log.csv 文件</p>";
        report << "</body></html>";
        
        std::cout << "\n测试报告已生成: intelligent_decision_test_report.html" << std::endl;
    }
};

// 主函数
int main() {
    IntelligentDecisionTestRunner test_runner;
    
    bool result = test_runner.runCriticalPlowResistanceTest();
    
    std::cout << "\n=== 测试完成 ===" << std::endl;
    std::cout << "结果: " << (result ? "✅ 通过" : "❌ 失败") << std::endl;
    
    return result ? 0 : 1;
}
