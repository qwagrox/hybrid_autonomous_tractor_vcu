# VCU仿真测试(SIL)系统架构设计

**版本**: 1.0  
**作者**: tangyong@stmail.ujs.edu.cn  
**日期**: 2024/09/19

## 1. 概述

本文档详细描述了混合动力自主拖拉机VCU系统的仿真测试架构。该仿真系统采用软件在环（Software-in-the-Loop, SIL）技术，能够在没有真实硬件的情况下，对VCU软件进行全面的功能验证和性能测试。

## 2. 仿真系统需求分析

### 2.1 功能需求

**核心仿真能力**：

- 动力总成仿真：发动机、电机、电池、CVT的动态响应模型
- 农具仿真：犁具、播种机、施肥机、喷药机的负载特性模型
- 环境仿真：地形坡度、土壤阻力、作物密度等环境因素
- 通信仿真：虚拟CAN总线，支持J1939和ISOBUS协议

**测试验证能力**：

- 故障注入：传感器故障、执行器卡死、通信中断等
- 场景管理：可配置的测试场景，支持YAML格式定义
- 数据记录：实时数据采集和后处理分析
- 自动化测试：批量测试执行和结果评估

### 2.2 性能需求

| 指标       | 要求    | 说明          |
| -------- | ----- | ----------- |
| **仿真步长** | 10ms  | 满足VCU实时控制需求 |
| **仿真精度** | ±5%   | 关键参数与真实系统误差 |
| **运行时间** | 实时或更快 | 支持长时间仿真测试   |
| **内存占用** | <2GB  | 在普通PC上运行    |
| **扩展性**  | 模块化   | 支持新增模型和测试场景 |

### 2.3 接口需求

**VCU软件接口**：

- 硬件抽象层（HAL）适配器
- CAN消息收发接口
- 传感器数据读取接口
- 执行器控制输出接口

**用户接口**：

- 配置文件接口（YAML格式）
- 命令行控制接口
- 数据可视化接口
- 测试报告生成接口

## 3. 系统架构设计

![仿真系统架构图](simulation_architecture.png)

### 3.1 分层架构

#### **第一层：测试与分析层**

负责测试场景管理、故障注入和结果分析。

**测试场景管理器**：

- 从YAML文件加载测试场景定义
- 控制仿真执行流程和时序
- 支持多场景批量执行

**故障注入模块**：

- 传感器故障：数值固定、噪声增加、信号丢失
- 执行器故障：响应延迟、输出限制、完全失效
- 通信故障：消息丢失、延迟、错误数据

**测试结果分析器**：

- 实时数据可视化
- 性能指标计算（燃油经济性、作业效率等）
- 自动化测试报告生成

#### **第二层：仿真核心层**

提供仿真运行时环境和基础服务。

**仿真主循环**：

```cpp
class SimulationCore {
public:
    void run() {
        while (simulation_time_ < total_duration_) {
            // 1. 更新环境模型
            environment_.update(simulation_time_);

            // 2. 执行VCU软件
            vcu_stack_.update(virtual_can_bus_);

            // 3. 更新物理模型
            updatePhysicalModels();

            // 4. 故障注入
            fault_injector_.apply(simulation_time_);

            // 5. 数据记录
            data_logger_.record(simulation_time_);

            // 6. 时间推进
            simulation_time_ += time_step_;
        }
    }
};
```

**虚拟CAN总线**：

- 模拟多路CAN总线（CAN0-CAN5）
- 支持J1939和ISOBUS协议栈
- 消息路由和总线负载仿真

**环境模型**：

- 地形模型：坡度变化、路面条件
- 作物模型：作物密度、生长状态
- 天气模型：温度、湿度、风速

#### **第三层：仿真模型层**

包含各个子系统的数学模型。

**动力总成模型**：

*发动机模型*：

```cpp
class EngineModel {
    void update(double throttle, double load_torque, double dt) {
        // 发动机转矩特性曲线
        double max_torque_at_speed = interpolate(torque_curve_, speed_);
        double requested_torque = throttle * max_torque_at_speed;

        // 动力学方程
        double net_torque = requested_torque - load_torque - friction_torque_;
        double angular_acceleration = net_torque / inertia_;
        speed_ += angular_acceleration * dt;

        // 燃油消耗计算
        fuel_consumption_ = calculateFuelConsumption(requested_torque, speed_);
    }
};
```

*电机模型*：

```cpp
class MotorModel {
    void update(double command, double load_torque, double dt) {
        // 电机转矩响应
        double target_torque = command * max_torque_;
        torque_ = firstOrderFilter(torque_, target_torque, time_constant_, dt);

        // 功率计算
        power_kw_ = torque_ * speed_ * 2 * M_PI / 60000;

        // 效率模型
        efficiency_ = calculateEfficiency(torque_, speed_);
    }
};
```

*电池模型*：

```cpp
class BatteryModel {
    void update(double power_draw_kw, double dt) {
        // 电池内阻模型
        double current = power_draw_kw * 1000 / voltage_;
        double voltage_drop = current * internal_resistance_;
        terminal_voltage_ = open_circuit_voltage_ - voltage_drop;

        // SOC计算
        double charge_used_ah = current * dt / 3600;
        soc_ -= (charge_used_ah / capacity_ah_) * 100;

        // 温度影响
        updateTemperatureEffects();
    }
};
```

**农具模型**：

*犁具模型*：

```cpp
class PlowModel {
    void update(double dt) {
        // 犁地阻力计算
        double soil_resistance = soil_type_factor_ * soil_moisture_factor_;
        double draft_force = working_width_ * working_depth_ * soil_resistance;
        load_torque_ = draft_force * effective_radius_;

        // 耕深控制响应
        actual_depth_ = firstOrderFilter(actual_depth_, target_depth_, 
                                       hydraulic_time_constant_, dt);
    }
};
```

*播种机模型*：

```cpp
class SeederModel {
    void update(double dt) {
        // 播种流量控制
        actual_flow_rate_ = target_flow_rate_ * seed_meter_efficiency_;

        // 播种深度控制
        actual_depth_ = firstOrderFilter(actual_depth_, target_depth_, 
                                       depth_control_time_constant_, dt);

        // 种子消耗计算
        seed_consumed_ += actual_flow_rate_ * dt;
    }
};
```

#### **第四层：被测对象层**

运行实际的VCU软件代码。

**VCU软件栈**：

- 直接运行您开发的VCU C++代码
- 通过HAL适配器与仿真模型交互
- 保持与真实系统相同的软件架构

**硬件抽象层适配器**：

```cpp
class HALAdapter {
public:
    // 将硬件读写操作转换为对仿真模型的调用
    uint16_t readADC(uint8_t channel) override {
        switch(channel) {
            case THROTTLE_POSITION_CHANNEL:
                return convertToADC(powertrain_model_.getThrottlePosition());
            case BATTERY_VOLTAGE_CHANNEL:
                return convertToADC(powertrain_model_.getBattery().getVoltage());
            // 其他传感器通道...
        }
    }

    void writePWM(uint8_t channel, uint16_t value) override {
        switch(channel) {
            case ENGINE_THROTTLE_CHANNEL:
                double throttle = convertFromPWM(value);
                powertrain_model_.setEngineThrottle(throttle);
                break;
            // 其他执行器通道...
        }
    }
};
```

### 3.2 数据流设计

**传感器数据流**：

```
物理模型 → 传感器仿真 → HAL适配器 → VCU软件
```

**控制指令流**：

```
VCU软件 → HAL适配器 → 执行器仿真 → 物理模型
```

**通信数据流**：

```
VCU软件 ↔ 虚拟CAN总线 ↔ 外部ECU仿真
```

### 3.3 时间同步机制

**固定步长仿真**：

- 仿真步长：10ms（与VCU控制周期一致）
- 所有模型在每个时间步同步更新
- 确保因果关系和时序正确性

**实时同步**：

```cpp
class RealTimeSync {
    void waitForNextStep() {
        auto target_time = start_time_ + 
                          std::chrono::milliseconds(step_count_ * 10);
        std::this_thread::sleep_until(target_time);
        step_count_++;
    }
};
```

## 4. 关键技术实现

### 4.1 模型精度与验证

**参数标定**：

- 基于真实拖拉机测试数据标定模型参数
- 使用最小二乘法优化模型精度
- 定期更新模型以反映硬件变化

**模型验证**：

- 与真实系统测试数据对比验证
- 关键工况点精度验证
- 长时间运行稳定性验证

### 4.2 故障建模

**传感器故障类型**：

```cpp
enum class SensorFaultType {
    STUCK_AT_VALUE,     // 数值固定
    NOISE_INJECTION,    // 噪声注入
    SIGNAL_LOSS,        // 信号丢失
    DRIFT,              // 参数漂移
    INTERMITTENT        // 间歇性故障
};
```

**执行器故障类型**：

```cpp
enum class ActuatorFaultType {
    RESPONSE_DELAY,     // 响应延迟
    OUTPUT_LIMITATION,  // 输出限制
    COMPLETE_FAILURE,   // 完全失效
    OSCILLATION         // 振荡
};
```

### 4.3 性能优化

**计算优化**：

- 使用查表法替代复杂数学计算
- 多线程并行计算独立模型
- 内存池管理减少动态分配

**数据管理**：

- 循环缓冲区存储历史数据
- 压缩算法减少存储空间
- 异步I/O提高数据记录性能

## 5. 扩展性设计

### 5.1 模型扩展

**新增农具模型**：

```cpp
class NewImplementModel : public BaseImplementModel {
public:
    void update(double dt) override {
        // 实现特定农具的动力学模型
    }

    double getLoadTorque() const override {
        // 返回农具负载转矩
    }
};
```

**新增传感器模型**：

```cpp
class NewSensorModel : public BaseSensorModel {
public:
    double readValue() override {
        // 实现传感器读数仿真
        return applyNoise(true_value_);
    }
};
```

### 5.2 协议扩展

**新增通信协议**：

```cpp
class NewProtocolAdapter : public BaseProtocolAdapter {
public:
    void processMessage(const CANMessage& msg) override {
        // 实现新协议的消息处理
    }

    CANMessage createMessage(const ControlCommand& cmd) override {
        // 实现新协议的消息生成
    }
};
```

## 6. 部署和使用

### 6.1 系统要求

**硬件要求**：

- CPU: Intel i5或同等性能
- 内存: 8GB RAM
- 存储: 10GB可用空间
- 操作系统: Windows 10/Linux Ubuntu 18.04+

**软件依赖**：

- C++17编译器（GCC 7.0+或MSVC 2017+）
- CMake 3.10+
- YAML-cpp库
- 可选：Python 3.7+（用于数据分析）

### 6.2 编译和运行

**编译步骤**：

```bash
mkdir build
cd build
cmake ..
make -j4
```

**运行仿真**：

```bash
./vcu_simulator --scenario scenarios/scenario1.yaml --duration 120
```

### 6.3 配置管理

**场景配置示例**：

```yaml
scenario_name: "Hill Climbing with Plow"
duration: 300  # seconds

environment:
  terrain:
    type: "hill"
    slope_angle: 15.0  # degrees
    length: 1000.0     # meters
  soil:
    type: "clay"
    moisture: 0.3      # 0-1
    resistance_factor: 1.5

implement:
  type: "plow"
  working_width: 3.0   # meters
  target_depth: 0.25   # meters

vcu_commands:
  - time: 0.0
    action: "start_engine"
  - time: 10.0
    action: "engage_implement"
  - time: 20.0
    action: "set_speed"
    value: 8.0  # km/h

faults:
  - type: "sensor_fault"
    target: "engine_speed"
    fault_type: "stuck_at_value"
    value: 1800.0
    start_time: 60.0
    duration: 30.0
```

## 7. 总结

本仿真系统为VCU开发提供了一个完整的测试验证平台，具有以下优势：

**技术优势**：

- 高精度的物理模型确保仿真结果可信
- 模块化架构支持灵活扩展
- 实时仿真能力满足控制系统测试需求

**经济优势**：

- 显著降低硬件测试成本
- 缩短开发周期
- 提高测试覆盖率

**安全优势**：

- 可安全测试危险工况
- 无硬件损坏风险
- 支持极限条件测试

通过充分利用这个仿真系统，可以在VCU硬件完成之前就验证控制算法的正确性，大大提高开发效率和产品质量。
