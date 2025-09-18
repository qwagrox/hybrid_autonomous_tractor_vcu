# 动力总成仿真数据输入示例

## 概述

本文档提供了VCU动力总成仿真系统的详细数据输入示例，包括发动机、电机、电池、CVT变速箱等各个子系统的参数配置和实时数据格式。

## 1. 发动机系统数据

### 1.1 发动机基础参数

```yaml
# engine_parameters.yaml
engine:
  basic_specs:
    model: "Cummins QSB6.7"
    displacement: 6.7          # L
    cylinders: 6
    max_power: 224             # kW @ 2200 RPM
    max_torque: 1356           # Nm @ 1500 RPM
    idle_speed: 800            # RPM
    rated_speed: 2200          # RPM
    redline_speed: 2500        # RPM
    
  fuel_system:
    injection_type: "Common Rail"
    fuel_pressure_max: 1800    # bar
    injector_count: 6
    fuel_tank_capacity: 400    # L
    
  cooling_system:
    coolant_capacity: 28       # L
    thermostat_temp: 82        # °C
    fan_on_temp: 95           # °C
    
  performance_curves:
    torque_curve:              # RPM: Nm
      800: 950
      1000: 1200
      1200: 1300
      1500: 1356               # Peak torque
      1800: 1320
      2000: 1280
      2200: 1220               # Rated power point
      2400: 1150
      2500: 1100
      
    power_curve:               # RPM: kW
      800: 80
      1000: 126
      1200: 163
      1500: 214
      1800: 248
      2000: 268
      2200: 280                # Peak power
      2400: 289
      2500: 288
      
    fuel_consumption_map:      # RPM: g/kWh
      800: 220
      1000: 205
      1200: 195
      1500: 190                # Best efficiency
      1800: 195
      2000: 205
      2200: 215
      2400: 230
      2500: 250
```

### 1.2 发动机实时数据输入格式

```cpp
// 发动机实时状态数据结构
struct EngineRealTimeData {
    // 基础运行参数
    double timestamp;              // 时间戳 (s)
    double engine_speed;           // 发动机转速 (RPM)
    double engine_load;            // 发动机负载 (%)
    double throttle_position;      // 油门位置 (0-100%)
    double fuel_flow_rate;         // 燃油流量 (L/h)
    double engine_torque;          // 输出扭矩 (Nm)
    double engine_power;           // 输出功率 (kW)
    
    // 温度参数
    double coolant_temp;           // 冷却液温度 (°C)
    double oil_temp;               // 机油温度 (°C)
    double exhaust_temp;           // 排气温度 (°C)
    double intake_air_temp;        // 进气温度 (°C)
    
    // 压力参数
    double oil_pressure;           // 机油压力 (bar)
    double fuel_pressure;          // 燃油压力 (bar)
    double boost_pressure;         // 增压压力 (bar)
    double exhaust_pressure;       // 排气背压 (bar)
    
    // 控制参数
    double injection_timing;       // 喷油正时 (°BTDC)
    double injection_duration;     // 喷油持续时间 (ms)
    double egr_valve_position;     // EGR阀位置 (%)
    double turbo_vane_position;    // 涡轮叶片位置 (%)
    
    // 状态标志
    bool engine_running;           // 发动机运行状态
    bool warming_up;               // 暖机状态
    bool overheating;              // 过热警告
    bool low_oil_pressure;         // 低油压警告
    uint32_t fault_codes;          // 故障码
};

// 示例数据
EngineRealTimeData engine_data_example = {
    .timestamp = 45.230,
    .engine_speed = 1850.5,
    .engine_load = 75.2,
    .throttle_position = 68.5,
    .fuel_flow_rate = 28.7,
    .engine_torque = 1285.3,
    .engine_power = 248.9,
    
    .coolant_temp = 89.5,
    .oil_temp = 95.2,
    .exhaust_temp = 485.7,
    .intake_air_temp = 45.8,
    
    .oil_pressure = 4.2,
    .fuel_pressure = 1650.0,
    .boost_pressure = 1.85,
    .exhaust_pressure = 0.15,
    
    .injection_timing = 8.5,
    .injection_duration = 2.8,
    .egr_valve_position = 15.2,
    .turbo_vane_position = 45.8,
    
    .engine_running = true,
    .warming_up = false,
    .overheating = false,
    .low_oil_pressure = false,
    .fault_codes = 0x00000000
};
```

## 2. 电机系统数据

### 2.1 电机基础参数

```yaml
# motor_parameters.yaml
motor:
  basic_specs:
    type: "Permanent Magnet Synchronous Motor"
    model: "Siemens 1PV5135"
    rated_power: 150           # kW
    peak_power: 200            # kW (短时)
    rated_torque: 800          # Nm
    peak_torque: 1200          # Nm (短时)
    rated_speed: 1800          # RPM
    max_speed: 4000            # RPM
    efficiency: 95.5           # %
    
  electrical_specs:
    rated_voltage: 650         # V DC
    max_voltage: 750           # V DC
    rated_current: 280         # A
    max_current: 400           # A (短时)
    power_factor: 0.95
    
  thermal_specs:
    max_winding_temp: 180      # °C
    max_magnet_temp: 120       # °C
    cooling_type: "Liquid"
    thermal_time_constant: 300 # s
    
  control_specs:
    control_type: "Vector Control"
    encoder_resolution: 2048   # pulses/rev
    switching_frequency: 10    # kHz
    response_time: 5           # ms
```

### 2.2 电机实时数据输入格式

```cpp
// 电机实时状态数据结构
struct MotorRealTimeData {
    // 基础运行参数
    double timestamp;              // 时间戳 (s)
    double motor_speed;            // 电机转速 (RPM)
    double motor_torque;           // 输出扭矩 (Nm)
    double motor_power;            // 输出功率 (kW)
    double efficiency;             // 当前效率 (%)
    
    // 电气参数
    double dc_voltage;             // 直流母线电压 (V)
    double dc_current;             // 直流电流 (A)
    double phase_current_u;        // U相电流 (A)
    double phase_current_v;        // V相电流 (A)
    double phase_current_w;        // W相电流 (A)
    double power_factor;           // 功率因数
    
    // 温度参数
    double winding_temp_u;         // U相绕组温度 (°C)
    double winding_temp_v;         // V相绕组温度 (°C)
    double winding_temp_w;         // W相绕组温度 (°C)
    double magnet_temp;            // 磁钢温度 (°C)
    double inverter_temp;          // 逆变器温度 (°C)
    
    // 控制参数
    double torque_command;         // 扭矩指令 (Nm)
    double speed_command;          // 转速指令 (RPM)
    double id_current;             // d轴电流 (A)
    double iq_current;             // q轴电流 (A)
    double rotor_position;         // 转子位置 (°)
    
    // 状态标志
    bool motor_enabled;            // 电机使能
    bool regeneration_active;      // 再生制动激活
    bool overtemperature;          // 过温保护
    bool overcurrent;              // 过流保护
    uint32_t fault_codes;          // 故障码
};

// 示例数据
MotorRealTimeData motor_data_example = {
    .timestamp = 45.230,
    .motor_speed = 2150.8,
    .motor_torque = 650.2,
    .motor_power = 146.7,
    .efficiency = 94.8,
    
    .dc_voltage = 685.5,
    .dc_current = 220.3,
    .phase_current_u = 185.7,
    .phase_current_v = 182.9,
    .phase_current_w = 188.1,
    .power_factor = 0.94,
    
    .winding_temp_u = 85.2,
    .winding_temp_v = 87.1,
    .winding_temp_w = 86.5,
    .magnet_temp = 78.9,
    .inverter_temp = 65.4,
    
    .torque_command = 650.0,
    .speed_command = 2150.0,
    .id_current = 25.8,
    .iq_current = 180.5,
    .rotor_position = 245.7,
    
    .motor_enabled = true,
    .regeneration_active = false,
    .overtemperature = false,
    .overcurrent = false,
    .fault_codes = 0x00000000
};
```

## 3. 电池系统数据

### 3.1 电池基础参数

```yaml
# battery_parameters.yaml
battery:
  basic_specs:
    type: "Lithium Iron Phosphate (LiFePO4)"
    nominal_voltage: 614.4     # V (192s2p configuration)
    capacity: 200              # Ah
    energy: 122.88             # kWh
    max_charge_current: 100    # A
    max_discharge_current: 300 # A
    
  cell_specs:
    cell_nominal_voltage: 3.2  # V
    cell_capacity: 100         # Ah
    series_count: 192          # 串联数量
    parallel_count: 2          # 并联数量
    
  thermal_specs:
    operating_temp_min: -20    # °C
    operating_temp_max: 60     # °C
    optimal_temp_min: 15       # °C
    optimal_temp_max: 35       # °C
    thermal_capacity: 1500     # J/K
    
  performance_specs:
    cycle_life: 6000           # cycles @ 80% DOD
    calendar_life: 15          # years
    round_trip_efficiency: 95  # %
    self_discharge_rate: 3     # %/month
```

### 3.2 电池实时数据输入格式

```cpp
// 电池实时状态数据结构
struct BatteryRealTimeData {
    // 基础电气参数
    double timestamp;              // 时间戳 (s)
    double pack_voltage;           // 电池包电压 (V)
    double pack_current;           // 电池包电流 (A, +充电/-放电)
    double pack_power;             // 电池包功率 (kW)
    double state_of_charge;        // 荷电状态 (%)
    double state_of_health;        // 健康状态 (%)
    double remaining_capacity;     // 剩余容量 (Ah)
    double energy_remaining;       // 剩余能量 (kWh)
    
    // 温度参数
    double max_cell_temp;          // 最高单体温度 (°C)
    double min_cell_temp;          // 最低单体温度 (°C)
    double avg_cell_temp;          // 平均单体温度 (°C)
    double coolant_inlet_temp;     // 冷却液入口温度 (°C)
    double coolant_outlet_temp;    // 冷却液出口温度 (°C)
    
    // 单体电压参数
    double max_cell_voltage;       // 最高单体电压 (V)
    double min_cell_voltage;       // 最低单体电压 (V)
    double avg_cell_voltage;       // 平均单体电压 (V)
    double voltage_difference;     // 电压差 (V)
    uint16_t max_voltage_cell_id;  // 最高电压单体ID
    uint16_t min_voltage_cell_id;  // 最低电压单体ID
    
    // 充放电能力
    double max_charge_power;       // 最大充电功率 (kW)
    double max_discharge_power;    // 最大放电功率 (kW)
    double charge_current_limit;   // 充电电流限制 (A)
    double discharge_current_limit;// 放电电流限制 (A)
    
    // 绝缘参数
    double positive_insulation;    // 正极绝缘阻抗 (kΩ)
    double negative_insulation;    // 负极绝缘阻抗 (kΩ)
    
    // 状态标志
    bool charging;                 // 充电状态
    bool discharging;              // 放电状态
    bool balancing;                // 均衡状态
    bool heating;                  // 加热状态
    bool cooling;                  // 冷却状态
    bool insulation_fault;         // 绝缘故障
    bool overtemperature;          // 过温保护
    bool undervoltage;             // 欠压保护
    bool overvoltage;              // 过压保护
    uint32_t fault_codes;          // 故障码
};

// 示例数据
BatteryRealTimeData battery_data_example = {
    .timestamp = 45.230,
    .pack_voltage = 645.8,
    .pack_current = -185.5,        // 放电
    .pack_power = -119.8,          // 放电功率
    .state_of_charge = 72.5,
    .state_of_health = 96.8,
    .remaining_capacity = 145.0,
    .energy_remaining = 89.1,
    
    .max_cell_temp = 32.8,
    .min_cell_temp = 28.5,
    .avg_cell_temp = 30.2,
    .coolant_inlet_temp = 25.8,
    .coolant_outlet_temp = 28.9,
    
    .max_cell_voltage = 3.385,
    .min_cell_voltage = 3.362,
    .avg_cell_voltage = 3.374,
    .voltage_difference = 0.023,
    .max_voltage_cell_id = 87,
    .min_voltage_cell_id = 156,
    
    .max_charge_power = 85.2,
    .max_discharge_power = 195.8,
    .charge_current_limit = 95.0,
    .discharge_current_limit = 285.0,
    
    .positive_insulation = 1250.5,
    .negative_insulation = 1180.8,
    
    .charging = false,
    .discharging = true,
    .balancing = false,
    .heating = false,
    .cooling = true,
    .insulation_fault = false,
    .overtemperature = false,
    .undervoltage = false,
    .overvoltage = false,
    .fault_codes = 0x00000000
};
```

## 4. CVT变速箱数据

### 4.1 CVT基础参数

```yaml
# cvt_parameters.yaml
cvt:
  basic_specs:
    type: "Hydrostatic CVT"
    model: "Bosch Rexroth VDT-MA"
    max_input_power: 300       # kW
    max_input_torque: 1800     # Nm
    max_input_speed: 2500      # RPM
    ratio_range_min: 0.5       # 最小传动比
    ratio_range_max: 3.0       # 最大传动比
    efficiency_max: 92         # %
    
  hydraulic_specs:
    system_pressure_max: 450   # bar
    displacement_pump_max: 125 # cc/rev
    displacement_motor_max: 250# cc/rev
    oil_capacity: 45           # L
    filtration_level: 6        # μm
    
  control_specs:
    response_time: 200         # ms
    ratio_change_rate_max: 2.0 # ratio/s
    pressure_control_accuracy: 2 # %
    speed_control_accuracy: 1  # %
```

### 4.2 CVT实时数据输入格式

```cpp
// CVT实时状态数据结构
struct CVTRealTimeData {
    // 基础传动参数
    double timestamp;              // 时间戳 (s)
    double input_speed;            // 输入转速 (RPM)
    double output_speed;           // 输出转速 (RPM)
    double input_torque;           // 输入扭矩 (Nm)
    double output_torque;          // 输出扭矩 (Nm)
    double transmission_ratio;     // 当前传动比
    double efficiency;             // 当前效率 (%)
    
    // 液压参数
    double system_pressure;        // 系统压力 (bar)
    double pump_displacement;      // 泵排量 (cc/rev)
    double motor_displacement;     // 马达排量 (cc/rev)
    double pump_swash_angle;       // 泵斜盘角度 (°)
    double motor_swash_angle;      // 马达斜盘角度 (°)
    double charge_pressure;        // 补油压力 (bar)
    
    // 温度参数
    double oil_temp_tank;          // 油箱温度 (°C)
    double oil_temp_system;        // 系统油温 (°C)
    double case_temp;              // 壳体温度 (°C)
    double cooler_inlet_temp;      // 冷却器入口温度 (°C)
    double cooler_outlet_temp;     // 冷却器出口温度 (°C)
    
    // 控制参数
    double ratio_command;          // 传动比指令
    double pressure_command;       // 压力指令 (bar)
    double pump_command;           // 泵控制指令 (%)
    double motor_command;          // 马达控制指令 (%)
    double servo_valve_position;   // 伺服阀位置 (%)
    
    // 滤清器状态
    double filter_pressure_drop;   // 滤清器压差 (bar)
    double filter_contamination;   // 污染度等级
    
    // 状态标志
    bool cvt_enabled;              // CVT使能
    bool forward_direction;        // 前进方向
    bool reverse_direction;        // 倒退方向
    bool neutral_position;         // 空档位置
    bool overheating;              // 过热保护
    bool low_pressure;             // 低压警告
    bool filter_clogged;           // 滤清器堵塞
    uint32_t fault_codes;          // 故障码
};

// 示例数据
CVTRealTimeData cvt_data_example = {
    .timestamp = 45.230,
    .input_speed = 1850.5,
    .output_speed = 1233.7,
    .input_torque = 1285.3,
    .output_torque = 1928.0,
    .transmission_ratio = 1.5,
    .efficiency = 89.5,
    
    .system_pressure = 385.2,
    .pump_displacement = 95.8,
    .motor_displacement = 180.5,
    .pump_swash_angle = 18.5,
    .motor_swash_angle = 22.8,
    .charge_pressure = 25.8,
    
    .oil_temp_tank = 65.8,
    .oil_temp_system = 78.5,
    .case_temp = 72.3,
    .cooler_inlet_temp = 78.5,
    .cooler_outlet_temp = 68.9,
    
    .ratio_command = 1.5,
    .pressure_command = 385.0,
    .pump_command = 76.5,
    .motor_command = 72.2,
    .servo_valve_position = 45.8,
    
    .filter_pressure_drop = 2.8,
    .filter_contamination = 18.5,
    
    .cvt_enabled = true,
    .forward_direction = true,
    .reverse_direction = false,
    .neutral_position = false,
    .overheating = false,
    .low_pressure = false,
    .filter_clogged = false,
    .fault_codes = 0x00000000
};
```

## 5. 综合动力总成数据

### 5.1 系统级数据结构

```cpp
// 综合动力总成数据结构
struct PowertrainSystemData {
    double timestamp;
    
    // 子系统数据
    EngineRealTimeData engine;
    MotorRealTimeData motor;
    BatteryRealTimeData battery;
    CVTRealTimeData cvt;
    
    // 系统级参数
    double total_output_power;     // 总输出功率 (kW)
    double total_output_torque;    // 总输出扭矩 (Nm)
    double fuel_consumption_rate;  // 燃油消耗率 (L/h)
    double energy_consumption_rate;// 电能消耗率 (kW)
    double system_efficiency;      // 系统总效率 (%)
    
    // 混合动力控制
    double power_split_ratio;      // 功率分配比例 (发动机/总功率)
    double regeneration_power;     // 再生制动功率 (kW)
    bool hybrid_mode_active;       // 混合动力模式激活
    bool engine_start_stop;        // 发动机启停状态
    
    // 系统状态
    uint8_t powertrain_mode;       // 动力总成模式
    bool system_ready;             // 系统就绪
    bool emergency_stop;           // 紧急停止
    uint32_t system_fault_codes;   // 系统故障码
};
```

### 5.2 数据输入文件格式

#### CSV格式示例
```csv
# powertrain_input_data.csv
Timestamp,EngineSpeed,EngineLoad,MotorSpeed,MotorTorque,BatterySOC,CVTRatio,SystemMode
0.000,800.0,15.5,0.0,0.0,85.0,1.0,0
0.010,805.2,16.8,0.0,0.0,85.0,1.0,0
0.020,812.5,18.2,0.0,0.0,84.9,1.0,0
...
45.230,1850.5,75.2,2150.8,650.2,72.5,1.5,2
45.240,1852.1,75.8,2148.5,655.8,72.4,1.5,2
```

#### JSON格式示例
```json
{
  "powertrain_data": {
    "timestamp": 45.230,
    "engine": {
      "speed": 1850.5,
      "load": 75.2,
      "torque": 1285.3,
      "fuel_flow": 28.7,
      "coolant_temp": 89.5,
      "oil_pressure": 4.2
    },
    "motor": {
      "speed": 2150.8,
      "torque": 650.2,
      "power": 146.7,
      "dc_voltage": 685.5,
      "efficiency": 94.8
    },
    "battery": {
      "soc": 72.5,
      "voltage": 645.8,
      "current": -185.5,
      "temperature": 30.2
    },
    "cvt": {
      "ratio": 1.5,
      "efficiency": 89.5,
      "oil_temp": 78.5,
      "pressure": 385.2
    }
  }
}
```

## 6. 仿真输入接口

### 6.1 C++接口示例

```cpp
class PowertrainDataInterface {
public:
    // 从文件加载数据
    bool loadFromCSV(const std::string& filename);
    bool loadFromJSON(const std::string& filename);
    
    // 实时数据输入
    void updateEngineData(const EngineRealTimeData& data);
    void updateMotorData(const MotorRealTimeData& data);
    void updateBatteryData(const BatteryRealTimeData& data);
    void updateCVTData(const CVTRealTimeData& data);
    
    // 数据获取
    PowertrainSystemData getCurrentData() const;
    std::vector<PowertrainSystemData> getHistoryData() const;
    
    // 数据验证
    bool validateData(const PowertrainSystemData& data) const;
    std::vector<std::string> getValidationErrors() const;
    
private:
    PowertrainSystemData current_data_;
    std::vector<PowertrainSystemData> history_data_;
    std::vector<std::string> validation_errors_;
};
```

### 6.2 使用示例

```cpp
// 使用示例
int main() {
    PowertrainDataInterface interface;
    
    // 方法1: 从文件加载
    if (interface.loadFromCSV("powertrain_input_data.csv")) {
        std::cout << "数据加载成功" << std::endl;
    }
    
    // 方法2: 实时数据输入
    EngineRealTimeData engine_data = {
        .timestamp = 45.230,
        .engine_speed = 1850.5,
        .engine_load = 75.2,
        // ... 其他参数
    };
    interface.updateEngineData(engine_data);
    
    // 获取当前数据
    PowertrainSystemData current = interface.getCurrentData();
    
    // 数据验证
    if (interface.validateData(current)) {
        std::cout << "数据验证通过" << std::endl;
    } else {
        auto errors = interface.getValidationErrors();
        for (const auto& error : errors) {
            std::cout << "验证错误: " << error << std::endl;
        }
    }
    
    return 0;
}
```

## 7. 数据验证规则

### 7.1 参数范围检查

```cpp
// 数据验证规则
struct ValidationRules {
    // 发动机参数范围
    struct {
        double speed_min = 0.0, speed_max = 3000.0;      // RPM
        double load_min = 0.0, load_max = 100.0;         // %
        double torque_min = 0.0, torque_max = 1500.0;    // Nm
        double temp_min = -40.0, temp_max = 120.0;       // °C
    } engine;
    
    // 电机参数范围
    struct {
        double speed_min = -5000.0, speed_max = 5000.0;  // RPM
        double torque_min = -1500.0, torque_max = 1500.0;// Nm
        double voltage_min = 400.0, voltage_max = 800.0; // V
        double temp_min = -40.0, temp_max = 200.0;       // °C
    } motor;
    
    // 电池参数范围
    struct {
        double soc_min = 0.0, soc_max = 100.0;           // %
        double voltage_min = 400.0, voltage_max = 750.0; // V
        double current_min = -400.0, current_max = 400.0;// A
        double temp_min = -30.0, temp_max = 70.0;        // °C
    } battery;
    
    // CVT参数范围
    struct {
        double ratio_min = 0.3, ratio_max = 4.0;         // ratio
        double pressure_min = 0.0, pressure_max = 500.0; // bar
        double temp_min = -40.0, temp_max = 150.0;       // °C
    } cvt;
};
```

这些数据输入示例为VCU仿真系统提供了完整的动力总成数据格式和接口规范，确保仿真的准确性和可靠性。
