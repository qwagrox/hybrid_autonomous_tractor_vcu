# HIL测试系统架构文档 v1.0

## 1. HIL系统概述

### 1.1 HIL vs SIL 对比

| 特性       | SIL (软件在环) | HIL (硬件在环)    |
| -------- | ---------- | ------------- |
| **被测对象** | VCU软件代码    | VCU真实硬件+软件    |
| **仿真环境** | 纯软件仿真      | 实时硬件仿真器       |
| **接口方式** | 函数调用       | 真实CAN/电气信号    |
| **测试精度** | 算法逻辑验证     | 硬件+软件完整验证     |
| **成本投入** | 低 ($5K)    | 高 ($50K-200K) |
| **测试覆盖** | 控制逻辑       | 硬件兼容性+时序+EMC  |

### 1.2 增强型HIL系统架构

本文档将详细介绍集成了CVT和电机仿真模块的增强型HIL（硬件在环）测试系统架构。此前的HIL系统已经包含了基础的动力总成和农具模型，而新架构的核心升级在于引入了高保真度的CVT（无级变速器）和电机（永磁同步电机）仿真能力，从而能够对混合动力拖拉机的VCU（车辆控制单元）进行更全面、更深入的测试。

**核心新增模块:**

* **dSPACE SCALEXIO CVT仿真模块**: 模拟CVT的机械、液压、热力学特性，以及与VCU的实时通信。
* **dSPACE SCALEXIO 电机仿真模块**: 模拟永磁同步电机的电气、机械、热力学特性，以及电机控制器的复杂行为。

这些新增模块使得HIL系统能够精确模拟混合动力系统在各种工况下的动态响应，为VCU控制策略的开发和验证提供了强大的支持。

![增强型HIL系统架构图](./hil_architecture_v2.png)

上图展示了集成了CVT和电机仿真模块后的增强型HIL系统架构。该架构的核心是dSPACE SCALEXIO实时仿真平台，它与测试PC和真实的VCU硬件相连，构成一个闭环的测试环境。

## 2. CVT仿真模块详解

CVT（无级变速器）是混合动力拖拉机的核心动力传递组件，其仿真的精确度直接影响HIL测试的有效性。dSPACE SCALEXIO平台提供了专门的CVT仿真模块，能够高保真地模拟CVT的复杂物理特性。

### 2.1 CVT仿真能力

dSPACE的CVT仿真方案覆盖了机械、液压、热力学和控制等多个物理域，能够模拟以下关键特性：

* **机械传动**: 模拟主动轮和从动轮的直径变化、皮带/链条的张力与滑移，以及由此决定的传动比变化。
* **液压系统**: 模拟控制传动比变化的液压泵、阀门和执行器的动态响应。
* **热力学特性**: 模拟CVT在运行过程中因摩擦和搅油产生的热量，以及冷却系统的散热效果，预测油温变化。
* **效率特性**: 基于实验数据建立效率Map，精确模拟CVT在不同转速和扭矩下的传动效率。

### 2.2 CVT软件仿真模型

在dSPACE SCALEXIO中，CVT的软件模型通常采用基于物理方程的建模方法，并结合从实验数据中提取的经验参数。一个典型的CVT模型包含以下子模型：

```cpp
class dSPACE_CVT_Model {
private:
    // 状态变量
    double current_ratio_;           // 当前传动比
    double target_ratio_;            // 目标传动比
    double primary_speed_;           // 主动轮转速
    double secondary_speed_;         // 从动轮转速
    double hydraulic_pressure_;      // 液压压力
    double oil_temperature_;         // 油温
    double belt_slip_;              // 皮带滑移率

public:
    // 主仿真循环 (1ms)
    void update(double dt) {
        // 1. 液压系统仿真
        updateHydraulicSystem(dt);

        // 2. 机械传动仿真
        updateMechanicalSystem(dt);

        // 3. 热力学仿真
        updateThermalSystem(dt);

        // 4. 效率计算
        updateEfficiency();

        // 5. 故障检测
        checkFaults();

        // 6. CAN消息更新
        updateCANMessages();
    }
    // ... 私有成员函数的实现 ...
};
```

### 2.3 VCU-CVT通信接口

VCU与CVT仿真模型之间的通信通过CAN总线进行，通常遵循J1939协议。VCU发送控制指令（如目标传动比、控制模式等），CVT模型则返回状态信息（如当前传动比、油温、故障代码等）。

**CVT控制消息 (VCU → CVT)**

* **PGN**: 0x18F00500 (示例)
* **数据**: 目标传动比, 控制模式, 使能标志, 最大扭矩限制

**CVT状态消息 (CVT → VCU)**

* **PGN**: 0x18F00600 (示例)
* **数据**: 当前传动比, 输入/输出转速, 油温, 液压压力, 故障代码, 效率

## 3. 电机仿真模块详解

电机系统是混合动力拖拉机的另一个核心组件，负责提供辅助动力、回收制动能量，并实现精确的扭矩控制。dSPACE SCALEXIO平台通过其高性能处理器和专用的I/O模块，能够对永磁同步电机（PMSM）及其控制器进行精确的实时仿真。

### 3.1 电机仿真能力

dSPACE的电机仿真解决方案能够模拟从底层电气动态到高层控制算法的完整电机系统，其主要能力包括：

* **电气系统仿真**: 在微秒级（μs）的时间尺度上，求解电机的dq轴电压和电流方程，精确模拟电机的电磁特性。
* **机械系统仿真**: 模拟电机的转子动力学，包括电磁扭矩、负载扭矩和转动惯量的相互作用，从而计算出转子的实时转速和位置。
* **热力学仿真**: 模拟电机在运行过程中的铜损和铁损，并根据热容和散热系数计算定子和转子的温度变化。
* **逆变器仿真**: 模拟PWM逆变器的开关行为，包括SVPWM调制、死区效应和开关损耗。
* **编码器仿真**: 生成高精度的编码器信号（A/B/Z相），作为电机控制器的位置和速度反馈。

### 3.2 电机软件仿真模型

电机仿真模型的核心是永磁同步电机的数学模型，通常在dSPACE SCALEXIO上以C++或Simulink实现。该模型通过实时求解微分方程来模拟电机的动态行为。

```cpp
class PMSM_Model {
private:
    // 电机参数
    double Rs_;          // 定子电阻 (Ω)
    double Ld_;          // d轴电感 (H)
    double Lq_;          // q轴电感 (H)
    double flux_pm_;     // 永磁体磁链 (Wb)
    double pole_pairs_;  // 极对数

    // 状态变量
    double id_, iq_;     // dq轴电流 (A)
    double omega_r_;     // 转子角速度 (rad/s)
    double theta_r_;     // 转子位置 (rad)
    double Te_;          // 电磁扭矩 (N·m)

public:
    // 电机方程求解 (1μs步长)
    void updateElectricalSystem(double dt, double vd, double vq) {
        // d轴和q轴电流方程的数值积分
        // ...
    }

    // 机械方程求解
    void updateMechanicalSystem(double dt) {
        // 转子动力学方程的数值积分
        // ...
    }

    // 温度模型
    void updateThermalModel(double dt) {
        // 热力学方程的数值积分
        // ...
    }
};
```

### 3.3 VCU-电机通信接口

VCU通过CAN总线向电机控制器（在HIL中为仿真模型）发送控制指令，并接收来自电机的状态反馈。这些通信报文通常包含以下信息：

**电机控制消息 (VCU → 电机)**

* **CAN ID**: 0x601 (示例)
* **数据**: 目标扭矩, 目标转速, 控制模式 (扭矩/速度模式), 使能标志

**电机状态消息 (电机 → VCU)**

* **CAN ID**: 0x611 (示例)
* **数据**: 实际扭矩, 实际转速, 直流母线电压, 电机温度, 故障代码

## 4. 增强型HIL测试环境搭建

搭建增强型HIL测试环境需要将VCU硬件、dSPACE SCALEXIO仿真平台以及新增的CVT和电机仿真模块进行物理连接和软件配置。

### 4.1 硬件连接方案

硬件连接的核心是将VCU的各个接口（CAN、I/O、电源）与dSPACE仿真器的对应通道相连。对于新增的电机仿真，还需要连接PWM信号和编码器信号。

#### VCU接口连接表示例

| VCU接口         | HIL仿真器接口          | 信号类型        | 用途          |
| ------------- | ----------------- | ----------- | ----------- |
| CAN0 (J1939)  | DS2654-CAN0       | CAN-FD      | 发动机通信       |
| CAN1 (J1939)  | DS2654-CAN1       | CAN-FD      | **电机通信**    |
| CAN2 (J1939)  | DS2654-CAN2       | CAN-FD      | **CVT通信**   |
| CAN3 (J1939)  | DS2654-CAN3       | CAN-FD      | 电池通信        |
| CAN4 (ISOBUS) | DS2654-CAN4       | CAN-FD      | 农具通信        |
| AI0-AI15      | DS2655-AI0-15     | 0-5V/4-20mA | 传感器信号       |
| AO0-AO7       | DS2655-AO0-7      | 0-10V       | 执行器控制       |
| **PWM0-PWM5** | **DS2655-PWM0-5** | **PWM**     | **逆变器门极驱动** |
| **ENC0**      | **DS2655-ENC0**   | **A/B/Z**   | **电机编码器反馈** |
| +12V/+24V     | DS2655-POWER      | 直流电源        | VCU供电       |
| GND           | DS2655-GND        | 接地          | 信号地         |

### 4.2 软件环境配置

软件环境配置主要在dSPACE ControlDesk中完成。需要加载包含CVT和电机模型的实时应用程序（SDF），并配置相应的CAN接口、I/O通道和数据记录。

```python
# ControlDesk自动化脚本 (Python)
import dspace.controldesk as cd

class EnhancedHILTestAutomation:
    def __init__(self):
        self.cd = cd.ControlDesk()
        self.experiment = None

    def setup_experiment(self):
        """设置增强型HIL实验"""
        # 加载包含CVT和电机模型的实时应用
        self.experiment = self.cd.load_experiment("Enhanced_VCU_HIL_Model.sdf")

        # 配置CAN接口 (包括CVT和电机)
        self.setup_can_interfaces()

        # 配置I/O接口 (包括PWM和编码器)
        self.setup_io_interfaces()

        # 配置数据记录
        self.setup_data_logging()

    def run_motor_test_scenario(self, scenario_file):
        """运行电机测试场景"""
        # ... (加载并执行特定于电机的测试场景)

    def run_cvt_test_scenario(self, scenario_file):
        """运行CVT测试场景"""
        # ... (加载并执行特定于CVT的测试场景)
```

## 5. 增强型HIL测试用例

集成了高保真CVT和电机模型后，HIL测试系统能够执行更复杂、更贴近真实工况的测试用例，从而对VCU的混合动力控制策略进行全面验证。

### 5.1 混合动力协同控制测试

这类测试的核心是验证VCU能否在不同工况下，最优地协调发动机、电机和CVT的工作，以达到最佳的动力性、经济性和驾驶性。

#### 场景1: 扭矩辅助（Torque Assist）测试

```yaml
# hil_torque_assist_test.yaml
test_name: "混合动力扭矩辅助功能测试"
description: "验证VCU在负载突增时，能否快速调用电机提供辅助扭矩"
duration: 120.0  # 秒

initial_conditions:
  engine_speed: 1600.0      # RPM, 发动机在经济转速区
  vehicle_speed: 7.0        # km/h
  motor_torque: 0.0         # Nm, 电机待命
  cvt_ratio: 1.8            # 较大的传动比以提供扭矩
  battery_soc: 75.0         # %

test_sequence:
  - time: 20.0
    action: "inject_implement_load_increase"
    parameters:
      load_increase_factor: 3.0 # 模拟犁地遇到坚硬土壤
      ramp_time: 2.0
    description: "注入农具负载突增"

  - time: 20.1
    action: "verify_vcu_response"
    parameters:
      max_motor_response_time: 0.1 # s, 电机响应时间
      min_motor_torque: 150.0      # Nm, 电机最小输出扭矩
      max_engine_speed_drop: 100.0 # RPM, 发动机转速跌落限制
    description: "验证VCU快速调用电机辅助扭矩"

  - time: 40.0
    action: "remove_implement_load"
    description: "负载恢复正常"

  - time: 40.1
    action: "verify_motor_disengagement"
    parameters:
      max_disengagement_time: 1.0 # s
      target_motor_torque: 0.0    # Nm
    description: "验证电机在负载恢复后平滑退出"

expected_results:
  engine_stability: "发动机转速波动在±100 RPM以内"
  power_delivery: "系统总输出扭矩平滑增加，无明显中断"
  battery_usage: "SOC下降符合电机输出功率"
```

### 5.2 能量回收测试

#### 场景2: 制动能量回收（Regenerative Braking）测试

```yaml
# hil_regen_braking_test.yaml
test_name: "制动能量回收功能测试"
description: "验证VCU在车辆减速或下坡时，能否有效控制电机回收能量"
duration: 90.0  # 秒

initial_conditions:
  vehicle_speed: 15.0       # km/h, 较高的行驶速度
  engine_status: "idle"     # 发动机怠速或关闭
  motor_torque: 0.0         # Nm
  battery_soc: 60.0         # %, 电池有充电空间

test_sequence:
  - time: 10.0
    action: "simulate_braking_request"
    parameters:
      braking_intensity: 0.5 # 50%强度的制动请求
    description: "模拟驾驶员踩下制动踏板"

  - time: 10.1
    action: "verify_motor_regeneration"
    parameters:
      max_regen_response_time: 0.2 # s
      min_regen_torque: -100.0     # Nm, 最小回收扭矩
      max_hydraulic_brake_usage: 0.1 # 尽可能少使用机械刹车
    description: "验证VCU优先使用电机进行能量回收"

  - time: 30.0
    action: "simulate_end_of_braking"
    description: "制动结束"

expected_results:
  battery_soc_increase: "SOC显著上升"
  vehicle_deceleration: "车辆平稳减速，符合驾驶员预期"
  energy_efficiency: "回收效率 > 85%"
```

### 5.3 故障注入与诊断测试

增强型HIL系统能够模拟CVT和电机自身的故障，以及它们与VCU之间通信的故障，从而全面测试VCU的故障诊断和处理能力。

#### 故障场景示例

* **CVT液压故障**: 模拟CVT液压泵压力不足或阀门卡死。预期VCU能够检测到该故障，并通过CAN总线报告故障代码，同时进入相应的跛行回家（Limp-Home）模式，如限制发动机扭矩和车速。
* **电机编码器信号丢失**: 模拟电机编码器信号中断。预期VCU能够切换到无传感器控制模式（如果支持），或者安全地停用电机并报告故障，以保证系统安全。
* **CAN通信中断**: 模拟VCU与CVT或电机控制器之间的CAN通信丢失。预期VCU能够检测到通信超时，并执行预设的安全策略，例如将CVT置于默认传动比，或将电机置于零扭矩输出。

## 6. HIL系统完整集成架构

为了更清晰地展示增强型HIL测试系统的完整架构，包括新增的CVT和电机仿真模块如何与现有系统集成，我们提供了以下详细的系统集成图表。

### 6.1 完整系统集成架构

![HIL系统完整集成架构](./hil_system_integration.png)

上图展示了增强型HIL测试系统的完整集成架构，其中包含了以下关键组件：

**测试控制层**：负责测试脚本的管理、数据记录、结果分析和故障注入控制。

**dSPACE SCALEXIO实时仿真平台**：系统的核心，包含处理器单元、各种仿真模型（发动机、CVT、电机、农具、环境、电池）、通信接口模块和I/O接口模块。

**信号调理板**：负责电平转换、滤波隔离、故障注入开关和状态指示。

**被测VCU硬件**：基于TC397TP的真实VCU硬件，包含所有必要的接口。

**电源系统**：提供可编程的12V/24V电源，并具备故障注入能力。

### 6.2 CVT和电机仿真详细连接

![CVT和电机仿真详细连接](./cvt_motor_simulation_detail.png)

上图详细展示了CVT和电机仿真模块的内部结构以及它们与VCU的连接方式：

**CVT仿真模块**包含机械模型、液压模型、热力学模型和效率模型，通过CAN接口、模拟输入/输出与VCU通信。

**电机仿真模块**包含电气模型、机械模型、热力学模型和逆变器模型，通过CAN接口、PWM输出、编码器输出和模拟输入与VCU通信。

这种详细的连接方式确保了仿真的高保真度，能够准确模拟真实系统中的各种物理现象和控制交互。
