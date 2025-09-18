// powertrain_data_generator.cpp
// 动力总成仿真数据生成器

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
#include <string>
#include <map>

// 动力总成数据结构
struct PowertrainData {
    double timestamp;
    
    // 发动机数据
    double engine_speed;           // RPM
    double engine_load;            // %
    double engine_torque;          // Nm
    double engine_power;           // kW
    double fuel_flow_rate;         // L/h
    double coolant_temp;           // °C
    double oil_pressure;           // bar
    double exhaust_temp;           // °C
    
    // 电机数据
    double motor_speed;            // RPM
    double motor_torque;           // Nm
    double motor_power;            // kW
    double motor_efficiency;       // %
    double dc_voltage;             // V
    double dc_current;             // A
    double motor_temp;             // °C
    
    // 电池数据
    double battery_soc;            // %
    double battery_voltage;        // V
    double battery_current;        // A
    double battery_power;          // kW
    double battery_temp;           // °C
    double max_charge_power;       // kW
    double max_discharge_power;    // kW
    
    // CVT数据
    double cvt_input_speed;        // RPM
    double cvt_output_speed;       // RPM
    double cvt_ratio;              // ratio
    double cvt_efficiency;         // %
    double cvt_oil_temp;           // °C
    double cvt_pressure;           // bar
    
    // 系统数据
    double total_output_power;     // kW
    double total_output_torque;    // Nm
    double system_efficiency;      // %
    int powertrain_mode;           // 0=发动机, 1=电机, 2=混合
    double power_split_ratio;      // 发动机功率比例
};

// 发动机特性曲线类
class EngineCharacteristics {
private:
    // 扭矩曲线数据点 (RPM, Nm)
    std::map<double, double> torque_curve_ = {
        {800, 950}, {1000, 1200}, {1200, 1300}, {1500, 1356},
        {1800, 1320}, {2000, 1280}, {2200, 1220}, {2400, 1150}, {2500, 1100}
    };
    
    // 燃油消耗曲线 (RPM, g/kWh)
    std::map<double, double> fuel_curve_ = {
        {800, 220}, {1000, 205}, {1200, 195}, {1500, 190},
        {1800, 195}, {2000, 205}, {2200, 215}, {2400, 230}, {2500, 250}
    };
    
public:
    double getTorque(double rpm, double load_percent) {
        double max_torque = interpolate(torque_curve_, rpm);
        return max_torque * (load_percent / 100.0);
    }
    
    double getFuelConsumption(double rpm, double power_kw) {
        double specific_consumption = interpolate(fuel_curve_, rpm);
        return (power_kw * specific_consumption) / 1000.0; // L/h
    }
    
private:
    double interpolate(const std::map<double, double>& curve, double x) {
        if (curve.empty()) return 0.0;
        
        auto it = curve.lower_bound(x);
        
        if (it == curve.begin()) return it->second;
        if (it == curve.end()) return curve.rbegin()->second;
        
        auto prev = std::prev(it);
        double x1 = prev->first, y1 = prev->second;
        double x2 = it->first, y2 = it->second;
        
        return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
    }
};

// 电机特性曲线类
class MotorCharacteristics {
public:
    double getTorque(double rpm, double torque_command) {
        double max_torque = getMaxTorque(rpm);
        return std::min(std::abs(torque_command), max_torque) * 
               (torque_command >= 0 ? 1.0 : -1.0);
    }
    
    double getEfficiency(double rpm, double torque) {
        double speed_factor = 1.0 - std::pow((rpm - 1800.0) / 3000.0, 2) * 0.1;
        double load_factor = 1.0 - std::pow((std::abs(torque) / 800.0 - 0.7), 2) * 0.15;
        return std::max(0.85, std::min(0.96, 0.95 * speed_factor * load_factor));
    }
    
private:
    double getMaxTorque(double rpm) {
        if (rpm <= 1800) return 800.0;
        else if (rpm <= 4000) return 800.0 * 1800.0 / rpm; // 恒功率区
        else return 360.0; // 最高转速限制
    }
};

// 电池模型类
class BatteryModel {
private:
    double capacity_ = 200.0;      // Ah
    double nominal_voltage_ = 614.4; // V
    double internal_resistance_ = 0.1; // Ω
    
public:
    double getVoltage(double soc, double current) {
        double ocv = getOpenCircuitVoltage(soc);
        return ocv - current * internal_resistance_;
    }
    
    double getMaxChargePower(double soc, double temp) {
        double temp_factor = getTemperatureFactor(temp);
        double soc_factor = (soc < 90) ? 1.0 : (100 - soc) / 10.0;
        return 100.0 * temp_factor * soc_factor; // kW
    }
    
    double getMaxDischargePower(double soc, double temp) {
        double temp_factor = getTemperatureFactor(temp);
        double soc_factor = (soc > 20) ? 1.0 : soc / 20.0;
        return 200.0 * temp_factor * soc_factor; // kW
    }
    
private:
    double getOpenCircuitVoltage(double soc) {
        // 简化的OCV-SOC曲线
        return nominal_voltage_ * (0.9 + 0.2 * soc / 100.0);
    }
    
    double getTemperatureFactor(double temp) {
        if (temp < 0) return 0.7;
        else if (temp < 15) return 0.8 + 0.2 * temp / 15.0;
        else if (temp <= 35) return 1.0;
        else if (temp <= 50) return 1.0 - 0.1 * (temp - 35) / 15.0;
        else return 0.6;
    }
};

// CVT模型类
class CVTModel {
public:
    double getOutputSpeed(double input_speed, double ratio) {
        return input_speed / ratio;
    }
    
    double getOutputTorque(double input_torque, double ratio, double efficiency) {
        return input_torque * ratio * (efficiency / 100.0);
    }
    
    double getEfficiency(double ratio, double power) {
        // CVT效率模型
        double ratio_factor = 1.0 - std::pow((ratio - 1.5) / 2.0, 2) * 0.1;
        double load_factor = 1.0 - std::pow((power / 200.0 - 0.6), 2) * 0.08;
        return std::max(0.75, std::min(0.92, 0.90 * ratio_factor * load_factor));
    }
};

// 数据生成器主类
class PowertrainDataGenerator {
private:
    EngineCharacteristics engine_char_;
    MotorCharacteristics motor_char_;
    BatteryModel battery_model_;
    CVTModel cvt_model_;
    
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
    
public:
    PowertrainDataGenerator() : rng_(std::random_device{}()), noise_dist_(0.0, 1.0) {}
    
    std::vector<PowertrainData> generateScenarioData(const std::string& scenario_name, 
                                                    double duration, double dt) {
        std::vector<PowertrainData> data;
        
        if (scenario_name == "normal_operation") {
            data = generateNormalOperation(duration, dt);
        } else if (scenario_name == "high_load_plowing") {
            data = generateHighLoadPlowing(duration, dt);
        } else if (scenario_name == "hybrid_mode_test") {
            data = generateHybridModeTest(duration, dt);
        } else if (scenario_name == "battery_charge_test") {
            data = generateBatteryChargeTest(duration, dt);
        } else {
            std::cout << "未知场景: " << scenario_name << std::endl;
        }
        
        return data;
    }
    
private:
    // 正常作业场景
    std::vector<PowertrainData> generateNormalOperation(double duration, double dt) {
        std::vector<PowertrainData> data;
        
        for (double t = 0; t <= duration; t += dt) {
            PowertrainData point;
            point.timestamp = t;
            
            // 发动机参数
            point.engine_speed = 1800 + 100 * sin(0.1 * t) + addNoise(10);
            point.engine_load = 65 + 10 * sin(0.05 * t) + addNoise(2);
            point.engine_torque = engine_char_.getTorque(point.engine_speed, point.engine_load);
            point.engine_power = point.engine_torque * point.engine_speed * 2 * M_PI / 60000; // kW
            point.fuel_flow_rate = engine_char_.getFuelConsumption(point.engine_speed, point.engine_power);
            point.coolant_temp = 85 + 5 * sin(0.02 * t) + addNoise(1);
            point.oil_pressure = 4.0 + 0.5 * sin(0.03 * t) + addNoise(0.1);
            point.exhaust_temp = 450 + 50 * sin(0.04 * t) + addNoise(10);
            
            // 电机参数（正常模式下电机不工作）
            point.motor_speed = 0;
            point.motor_torque = 0;
            point.motor_power = 0;
            point.motor_efficiency = 0;
            point.dc_voltage = 650 + addNoise(5);
            point.dc_current = 0;
            point.motor_temp = 25 + addNoise(2);
            
            // 电池参数
            point.battery_soc = 85 - t / duration * 5 + addNoise(0.5); // 缓慢下降
            point.battery_voltage = battery_model_.getVoltage(point.battery_soc, point.dc_current);
            point.battery_current = point.dc_current;
            point.battery_power = point.battery_voltage * point.battery_current / 1000;
            point.battery_temp = 30 + 3 * sin(0.01 * t) + addNoise(1);
            point.max_charge_power = battery_model_.getMaxChargePower(point.battery_soc, point.battery_temp);
            point.max_discharge_power = battery_model_.getMaxDischargePower(point.battery_soc, point.battery_temp);
            
            // CVT参数
            point.cvt_ratio = 1.5 + 0.3 * sin(0.08 * t) + addNoise(0.05);
            point.cvt_input_speed = point.engine_speed;
            point.cvt_output_speed = cvt_model_.getOutputSpeed(point.cvt_input_speed, point.cvt_ratio);
            point.cvt_efficiency = cvt_model_.getEfficiency(point.cvt_ratio, point.engine_power);
            point.cvt_oil_temp = 75 + 8 * sin(0.02 * t) + addNoise(2);
            point.cvt_pressure = 350 + 30 * sin(0.06 * t) + addNoise(5);
            
            // 系统参数
            point.total_output_power = point.engine_power * (point.cvt_efficiency / 100);
            point.total_output_torque = cvt_model_.getOutputTorque(point.engine_torque, 
                                                                 point.cvt_ratio, point.cvt_efficiency);
            point.system_efficiency = point.cvt_efficiency;
            point.powertrain_mode = 0; // 纯发动机模式
            point.power_split_ratio = 1.0;
            
            data.push_back(point);
        }
        
        return data;
    }
    
    // 高负载犁地场景
    std::vector<PowertrainData> generateHighLoadPlowing(double duration, double dt) {
        std::vector<PowertrainData> data;
        
        for (double t = 0; t <= duration; t += dt) {
            PowertrainData point;
            point.timestamp = t;
            
            // 模拟阻力变化
            double load_factor = 1.0;
            if (t >= 30 && t <= 60) {
                // 高阻力期
                load_factor = 1.0 + 3.5 * (1.0 - exp(-5.0 * (t - 30) / 5.0));
                if (t >= 35) load_factor = 4.5 + 0.5 * sin(2.0 * M_PI * (t - 35));
                if (t >= 45) load_factor = 4.5 * exp(-2.0 * (t - 45) / 15.0) + 1.0;
            }
            
            // 发动机参数
            point.engine_speed = 1850 + 50 * sin(0.1 * t) + addNoise(15);
            point.engine_load = std::min(95.0, 70 + 25 * load_factor + addNoise(3));
            point.engine_torque = engine_char_.getTorque(point.engine_speed, point.engine_load);
            point.engine_power = point.engine_torque * point.engine_speed * 2 * M_PI / 60000;
            point.fuel_flow_rate = engine_char_.getFuelConsumption(point.engine_speed, point.engine_power);
            point.coolant_temp = 90 + 8 * load_factor + addNoise(2);
            point.oil_pressure = 4.2 + 0.3 * load_factor + addNoise(0.1);
            point.exhaust_temp = 480 + 80 * load_factor + addNoise(15);
            
            // 电机参数（高负载时启用混合动力）
            bool hybrid_active = (load_factor > 2.0);
            if (hybrid_active) {
                point.motor_speed = point.engine_speed * 1.2;
                point.motor_torque = std::min(800.0, 200 * (load_factor - 1.5));
                point.motor_efficiency = motor_char_.getEfficiency(point.motor_speed, point.motor_torque);
                point.motor_power = point.motor_torque * point.motor_speed * 2 * M_PI / 60000;
                point.dc_current = -point.motor_power * 1000 / point.dc_voltage; // 放电
                point.motor_temp = 45 + 20 * (load_factor - 2.0) + addNoise(3);
            } else {
                point.motor_speed = 0;
                point.motor_torque = 0;
                point.motor_power = 0;
                point.motor_efficiency = 0;
                point.dc_current = 0;
                point.motor_temp = 30 + addNoise(2);
            }
            
            point.dc_voltage = battery_model_.getVoltage(point.battery_soc, point.dc_current);
            
            // 电池参数
            double soc_decrease_rate = hybrid_active ? 0.1 : 0.02; // %/min
            point.battery_soc = std::max(20.0, 85 - t * soc_decrease_rate / 60 + addNoise(0.5));
            point.battery_voltage = point.dc_voltage;
            point.battery_current = point.dc_current;
            point.battery_power = point.battery_voltage * point.battery_current / 1000;
            point.battery_temp = 32 + 8 * load_factor + addNoise(1.5);
            point.max_charge_power = battery_model_.getMaxChargePower(point.battery_soc, point.battery_temp);
            point.max_discharge_power = battery_model_.getMaxDischargePower(point.battery_soc, point.battery_temp);
            
            // CVT参数
            point.cvt_ratio = 1.2 + 0.8 / load_factor + addNoise(0.03); // 高负载时降低传动比
            point.cvt_input_speed = point.engine_speed;
            point.cvt_output_speed = cvt_model_.getOutputSpeed(point.cvt_input_speed, point.cvt_ratio);
            point.cvt_efficiency = cvt_model_.getEfficiency(point.cvt_ratio, 
                                                          point.engine_power + point.motor_power);
            point.cvt_oil_temp = 80 + 15 * load_factor + addNoise(3);
            point.cvt_pressure = 380 + 50 * load_factor + addNoise(8);
            
            // 系统参数
            point.total_output_power = (point.engine_power + point.motor_power) * (point.cvt_efficiency / 100);
            point.total_output_torque = cvt_model_.getOutputTorque(
                point.engine_torque + point.motor_torque, point.cvt_ratio, point.cvt_efficiency);
            point.system_efficiency = point.cvt_efficiency * 
                (hybrid_active ? (point.motor_efficiency / 100) : 1.0);
            point.powertrain_mode = hybrid_active ? 2 : 0; // 混合模式或发动机模式
            point.power_split_ratio = point.engine_power / (point.engine_power + point.motor_power + 1e-6);
            
            data.push_back(point);
        }
        
        return data;
    }
    
    // 混合动力模式测试场景
    std::vector<PowertrainData> generateHybridModeTest(double duration, double dt) {
        std::vector<PowertrainData> data;
        
        for (double t = 0; t <= duration; t += dt) {
            PowertrainData point;
            point.timestamp = t;
            
            // 功率需求变化
            double power_demand = 150 + 100 * sin(0.05 * t) + 50 * sin(0.2 * t);
            
            // 功率分配策略
            double engine_power_ratio = 0.6 + 0.3 * sin(0.03 * t);
            double motor_power_ratio = 1.0 - engine_power_ratio;
            
            // 发动机参数
            double target_engine_power = power_demand * engine_power_ratio;
            point.engine_speed = 1600 + 400 * (target_engine_power / 200) + addNoise(20);
            point.engine_load = std::min(90.0, target_engine_power / 2.5 + addNoise(3));
            point.engine_torque = engine_char_.getTorque(point.engine_speed, point.engine_load);
            point.engine_power = point.engine_torque * point.engine_speed * 2 * M_PI / 60000;
            point.fuel_flow_rate = engine_char_.getFuelConsumption(point.engine_speed, point.engine_power);
            point.coolant_temp = 88 + 5 * sin(0.02 * t) + addNoise(1.5);
            point.oil_pressure = 4.1 + 0.4 * sin(0.04 * t) + addNoise(0.1);
            point.exhaust_temp = 460 + 40 * sin(0.03 * t) + addNoise(12);
            
            // 电机参数
            double target_motor_power = power_demand * motor_power_ratio;
            point.motor_speed = point.engine_speed * 1.1;
            point.motor_torque = std::min(800.0, target_motor_power * 60000 / 
                                        (point.motor_speed * 2 * M_PI));
            point.motor_efficiency = motor_char_.getEfficiency(point.motor_speed, point.motor_torque);
            point.motor_power = point.motor_torque * point.motor_speed * 2 * M_PI / 60000;
            point.dc_voltage = 660 + 20 * sin(0.1 * t) + addNoise(5);
            point.dc_current = -point.motor_power * 1000 / point.dc_voltage;
            point.motor_temp = 50 + 15 * (point.motor_power / 150) + addNoise(3);
            
            // 电池参数
            double soc_change_rate = -point.motor_power * 0.05; // %/min
            static double cumulative_soc_change = 0;
            cumulative_soc_change += soc_change_rate * dt / 60;
            point.battery_soc = std::max(15.0, std::min(95.0, 80 + cumulative_soc_change + addNoise(0.3)));
            point.battery_voltage = battery_model_.getVoltage(point.battery_soc, point.dc_current);
            point.battery_current = point.dc_current;
            point.battery_power = point.battery_voltage * point.battery_current / 1000;
            point.battery_temp = 35 + 8 * (std::abs(point.motor_power) / 150) + addNoise(1.5);
            point.max_charge_power = battery_model_.getMaxChargePower(point.battery_soc, point.battery_temp);
            point.max_discharge_power = battery_model_.getMaxDischargePower(point.battery_soc, point.battery_temp);
            
            // CVT参数
            point.cvt_ratio = 1.4 + 0.4 * sin(0.06 * t) + addNoise(0.04);
            point.cvt_input_speed = point.engine_speed;
            point.cvt_output_speed = cvt_model_.getOutputSpeed(point.cvt_input_speed, point.cvt_ratio);
            point.cvt_efficiency = cvt_model_.getEfficiency(point.cvt_ratio, 
                                                          point.engine_power + point.motor_power);
            point.cvt_oil_temp = 78 + 10 * sin(0.02 * t) + addNoise(2.5);
            point.cvt_pressure = 360 + 40 * sin(0.05 * t) + addNoise(6);
            
            // 系统参数
            point.total_output_power = (point.engine_power + point.motor_power) * (point.cvt_efficiency / 100);
            point.total_output_torque = cvt_model_.getOutputTorque(
                point.engine_torque + point.motor_torque, point.cvt_ratio, point.cvt_efficiency);
            point.system_efficiency = point.cvt_efficiency * (point.motor_efficiency / 100);
            point.powertrain_mode = 2; // 混合模式
            point.power_split_ratio = point.engine_power / (point.engine_power + point.motor_power + 1e-6);
            
            data.push_back(point);
        }
        
        return data;
    }
    
    // 电池充电测试场景
    std::vector<PowertrainData> generateBatteryChargeTest(double duration, double dt) {
        std::vector<PowertrainData> data;
        
        for (double t = 0; t <= duration; t += dt) {
            PowertrainData point;
            point.timestamp = t;
            
            // 发动机参数（充电模式）
            point.engine_speed = 1500 + addNoise(10);
            point.engine_load = 45 + 10 * sin(0.1 * t) + addNoise(2);
            point.engine_torque = engine_char_.getTorque(point.engine_speed, point.engine_load);
            point.engine_power = point.engine_torque * point.engine_speed * 2 * M_PI / 60000;
            point.fuel_flow_rate = engine_char_.getFuelConsumption(point.engine_speed, point.engine_power);
            point.coolant_temp = 82 + 3 * sin(0.02 * t) + addNoise(1);
            point.oil_pressure = 3.8 + 0.2 * sin(0.03 * t) + addNoise(0.08);
            point.exhaust_temp = 420 + 30 * sin(0.04 * t) + addNoise(8);
            
            // 电机参数（发电模式）
            point.motor_speed = point.engine_speed * 1.0;
            point.motor_torque = -300 - 100 * sin(0.08 * t); // 负扭矩表示发电
            point.motor_efficiency = motor_char_.getEfficiency(point.motor_speed, std::abs(point.motor_torque));
            point.motor_power = point.motor_torque * point.motor_speed * 2 * M_PI / 60000; // 负功率
            point.dc_voltage = 670 + 15 * sin(0.05 * t) + addNoise(4);
            point.dc_current = std::abs(point.motor_power) * 1000 / point.dc_voltage; // 充电电流
            point.motor_temp = 40 + 10 * (std::abs(point.motor_power) / 100) + addNoise(2);
            
            // 电池参数（充电中）
            double soc_increase_rate = std::abs(point.motor_power) * 0.08; // %/min
            static double cumulative_soc_change = 0;
            cumulative_soc_change += soc_increase_rate * dt / 60;
            point.battery_soc = std::min(100.0, 60 + cumulative_soc_change + addNoise(0.4));
            point.battery_voltage = battery_model_.getVoltage(point.battery_soc, point.dc_current);
            point.battery_current = point.dc_current;
            point.battery_power = point.battery_voltage * point.battery_current / 1000;
            point.battery_temp = 28 + 5 * (point.battery_power / 50) + addNoise(1);
            point.max_charge_power = battery_model_.getMaxChargePower(point.battery_soc, point.battery_temp);
            point.max_discharge_power = battery_model_.getMaxDischargePower(point.battery_soc, point.battery_temp);
            
            // CVT参数
            point.cvt_ratio = 1.8 + addNoise(0.02);
            point.cvt_input_speed = point.engine_speed;
            point.cvt_output_speed = cvt_model_.getOutputSpeed(point.cvt_input_speed, point.cvt_ratio);
            point.cvt_efficiency = cvt_model_.getEfficiency(point.cvt_ratio, 
                                                          point.engine_power - std::abs(point.motor_power));
            point.cvt_oil_temp = 70 + 5 * sin(0.02 * t) + addNoise(1.5);
            point.cvt_pressure = 320 + 20 * sin(0.04 * t) + addNoise(4);
            
            // 系统参数
            point.total_output_power = (point.engine_power - std::abs(point.motor_power)) * 
                                     (point.cvt_efficiency / 100);
            point.total_output_torque = cvt_model_.getOutputTorque(
                point.engine_torque + point.motor_torque, point.cvt_ratio, point.cvt_efficiency);
            point.system_efficiency = point.cvt_efficiency * (point.motor_efficiency / 100);
            point.powertrain_mode = 1; // 发电模式
            point.power_split_ratio = 1.0; // 全部来自发动机
            
            data.push_back(point);
        }
        
        return data;
    }
    
    double addNoise(double std_dev) {
        return noise_dist_(rng_) * std_dev;
    }
};

// 数据输出类
class DataExporter {
public:
    static void exportToCSV(const std::vector<PowertrainData>& data, 
                           const std::string& filename) {
        std::ofstream file(filename);
        
        // 写入表头
        file << "Timestamp,EngineSpeed,EngineLoad,EngineTorque,EnginePower,FuelFlowRate,"
             << "CoolantTemp,OilPressure,ExhaustTemp,MotorSpeed,MotorTorque,MotorPower,"
             << "MotorEfficiency,DCVoltage,DCCurrent,MotorTemp,BatterySOC,BatteryVoltage,"
             << "BatteryCurrent,BatteryPower,BatteryTemp,MaxChargePower,MaxDischargePower,"
             << "CVTRatio,CVTInputSpeed,CVTOutputSpeed,CVTEfficiency,CVTOilTemp,CVTPressure,"
             << "TotalOutputPower,TotalOutputTorque,SystemEfficiency,PowertrainMode,PowerSplitRatio"
             << std::endl;
        
        // 写入数据
        for (const auto& point : data) {
            file << std::fixed << std::setprecision(3)
                 << point.timestamp << ","
                 << point.engine_speed << ","
                 << point.engine_load << ","
                 << point.engine_torque << ","
                 << point.engine_power << ","
                 << point.fuel_flow_rate << ","
                 << point.coolant_temp << ","
                 << point.oil_pressure << ","
                 << point.exhaust_temp << ","
                 << point.motor_speed << ","
                 << point.motor_torque << ","
                 << point.motor_power << ","
                 << point.motor_efficiency << ","
                 << point.dc_voltage << ","
                 << point.dc_current << ","
                 << point.motor_temp << ","
                 << point.battery_soc << ","
                 << point.battery_voltage << ","
                 << point.battery_current << ","
                 << point.battery_power << ","
                 << point.battery_temp << ","
                 << point.max_charge_power << ","
                 << point.max_discharge_power << ","
                 << point.cvt_ratio << ","
                 << point.cvt_input_speed << ","
                 << point.cvt_output_speed << ","
                 << point.cvt_efficiency << ","
                 << point.cvt_oil_temp << ","
                 << point.cvt_pressure << ","
                 << point.total_output_power << ","
                 << point.total_output_torque << ","
                 << point.system_efficiency << ","
                 << point.powertrain_mode << ","
                 << point.power_split_ratio
                 << std::endl;
        }
        
        file.close();
        std::cout << "数据已导出到: " << filename << std::endl;
    }
    
    static void printSummary(const std::vector<PowertrainData>& data) {
        if (data.empty()) return;
        
        std::cout << "\n=== 数据生成摘要 ===" << std::endl;
        std::cout << "数据点数量: " << data.size() << std::endl;
        std::cout << "时间范围: " << data.front().timestamp << "s - " 
                  << data.back().timestamp << "s" << std::endl;
        
        // 计算统计信息
        double avg_engine_power = 0, avg_motor_power = 0, avg_battery_soc = 0;
        double max_engine_power = 0, max_motor_power = 0;
        
        for (const auto& point : data) {
            avg_engine_power += point.engine_power;
            avg_motor_power += point.motor_power;
            avg_battery_soc += point.battery_soc;
            max_engine_power = std::max(max_engine_power, point.engine_power);
            max_motor_power = std::max(max_motor_power, std::abs(point.motor_power));
        }
        
        avg_engine_power /= data.size();
        avg_motor_power /= data.size();
        avg_battery_soc /= data.size();
        
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "平均发动机功率: " << avg_engine_power << " kW" << std::endl;
        std::cout << "平均电机功率: " << avg_motor_power << " kW" << std::endl;
        std::cout << "平均电池SOC: " << avg_battery_soc << " %" << std::endl;
        std::cout << "最大发动机功率: " << max_engine_power << " kW" << std::endl;
        std::cout << "最大电机功率: " << max_motor_power << " kW" << std::endl;
    }
};

// 主函数
int main(int argc, char* argv[]) {
    std::cout << "VCU动力总成仿真数据生成器" << std::endl;
    std::cout << "版本: 1.0" << std::endl;
    std::cout << "=" * 40 << std::endl;
    
    PowertrainDataGenerator generator;
    
    // 可用场景
    std::vector<std::string> scenarios = {
        "normal_operation",
        "high_load_plowing", 
        "hybrid_mode_test",
        "battery_charge_test"
    };
    
    std::string scenario = "normal_operation";
    double duration = 180.0; // 默认3分钟
    double dt = 0.1;         // 默认100ms
    
    // 解析命令行参数
    if (argc >= 2) scenario = argv[1];
    if (argc >= 3) duration = std::stod(argv[2]);
    if (argc >= 4) dt = std::stod(argv[3]);
    
    std::cout << "生成场景: " << scenario << std::endl;
    std::cout << "持续时间: " << duration << " 秒" << std::endl;
    std::cout << "采样间隔: " << dt << " 秒" << std::endl;
    std::cout << std::endl;
    
    // 生成数据
    auto data = generator.generateScenarioData(scenario, duration, dt);
    
    if (data.empty()) {
        std::cout << "数据生成失败！" << std::endl;
        std::cout << "可用场景: ";
        for (const auto& s : scenarios) {
            std::cout << s << " ";
        }
        std::cout << std::endl;
        return 1;
    }
    
    // 导出数据
    std::string filename = "powertrain_data_" + scenario + ".csv";
    DataExporter::exportToCSV(data, filename);
    
    // 打印摘要
    DataExporter::printSummary(data);
    
    std::cout << "\n使用方法:" << std::endl;
    std::cout << "./powertrain_data_generator [场景] [时长] [间隔]" << std::endl;
    std::cout << "示例: ./powertrain_data_generator high_load_plowing 300 0.05" << std::endl;
    
    return 0;
}
