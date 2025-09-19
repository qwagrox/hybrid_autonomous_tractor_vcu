# VCU核心类型定义文件解析

**文件路径**: `include/vcu_core_types.hpp`  
**文件作用**: 定义混合动力自主拖拉机VCU系统中所有核心数据结构、枚举类型和配置参数  
**分析日期**: 2025年/09/19  
**作者**: tangyong@stmail.ujs.edu.cn  

---

## 📋 目录结构

1. [文件头部和依赖项](#1-文件头部和依赖项)
2. [命名空间和类型别名](#2-命名空间和类型别名)
3. [系统状态枚举](#3-系统状态枚举)
4. [驱动模式枚举](#4-驱动模式枚举)
5. [CVT制造商枚举](#5-cvt制造商枚举)
6. [负载变化类型枚举](#6-负载变化类型枚举)
7. [故障相关枚举](#7-故障相关枚举)
8. [核心数据结构](#8-核心数据结构)
9. [农具控制类型](#9-农具控制类型)

---

## 1. 文件头部和依赖项

### 第1-9行：预处理指令和头文件包含

```cpp
// include/vcu_core_types.hpp
#pragma once
#include <eigen3/Eigen/Dense>
#include <chrono>
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cstdint>
```

#### 逐行解析

**第1行：文件标识注释**

```cpp
// include/vcu_core_types.hpp
```

- **作用**: 标识文件路径，便于代码维护和调试
- **最佳实践**: 在每个头文件开头注明文件路径，特别是在大型项目中

**第2行：头文件保护**

```cpp
#pragma once
```

- **作用**: 防止头文件被重复包含，避免重定义错误
- **现代C++选择**: 相比传统的 `#ifndef/#define/#endif` 保护，`#pragma once` 更简洁
- **编译器支持**: 所有现代编译器都支持此指令
- **优势**: 
  - 代码更简洁
  - 避免宏名冲突
  - 编译器优化更好

**第3行：Eigen数学库**

```cpp
#include <eigen3/Eigen/Dense>
```

- **库介绍**: Eigen是C++模板库，用于线性代数、矩阵、向量运算
- **选择原因**:
  - 高性能：大量使用模板和编译期优化
  - 广泛应用：机器人学、计算机视觉、控制系统的标准选择
  - 接口友好：类似MATLAB的语法
- **在VCU中的应用**:
  - 3D位置和速度向量计算
  - 姿态矩阵运算（旋转、变换）
  - 卡尔曼滤波中的协方差矩阵运算
  - 控制算法中的矩阵运算

**第4行：时间处理库**

```cpp
#include <chrono>
```

- **库介绍**: C++11引入的标准时间库
- **功能**: 提供高精度时间测量和时间点表示
- **在VCU中的应用**:
  - 控制循环的精确计时
  - 传感器数据的时间戳
  - 性能统计和延迟测量
  - 超时检测和故障诊断

**第5行：固定大小数组**

```cpp
#include <array>
```

- **库介绍**: C++11的固定大小数组容器
- **优势**: 相比C风格数组，提供边界检查和迭代器支持
- **在VCU中的应用**: 存储固定数量的数据，如四个车轮的转速

**第6行：动态数组**

```cpp
#include <vector>
```

- **库介绍**: C++标准库的动态数组容器
- **特点**: 自动内存管理，支持动态扩容
- **在VCU中的应用**: 
  - 存储可变长度的预测数据
  - 故障历史记录
  - 路径规划点集合

**第7行：字符串处理**

```cpp
#include <string>
```

- **库介绍**: C++标准字符串类
- **在VCU中的应用**:
  - 故障描述信息
  - 配置参数名称
  - 日志信息记录

**第8行：智能指针**

```cpp
#include <memory>
```

- **库介绍**: C++11智能指针库
- **现代C++内存管理**: 自动内存管理，避免内存泄漏
- **在VCU中的应用**: 管理动态分配的对象生命周期

**第9行：固定宽度整数**

```cpp
#include <cstdint>
```

- **库介绍**: 提供固定宽度的整数类型
- **重要性**: 确保跨平台的数据类型一致性
- **常用类型**:
  - `uint8_t`: 8位无符号整数 (0-255)
  - `uint16_t`: 16位无符号整数 (0-65535)
  - `uint32_t`: 32位无符号整数
- **在嵌入式系统中的意义**: 精确控制内存使用和CAN总线数据格式

---

## 2. 命名空间和类型别名

### 第11-16行：命名空间定义和类型别名

```cpp
namespace VCUCore {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Timestamp = std::chrono::nanoseconds;
```

**第11行：命名空间开始**

```cpp
namespace VCUCore {
```

- **作用**: 创建VCUCore命名空间，避免命名冲突
- **设计原则**: 将所有VCU相关的类型封装在统一命名空间下
- **好处**:
  - 避免与其他库的类型名冲突
  - 代码组织更清晰
  - 便于模块化开发

**第13行：3D向量类型别名**

```cpp
using Vector3d = Eigen::Vector3d;
```

- **原始类型**: `Eigen::Vector3d` 是3个double元素的向量
- **使用场景**:
  - GNSS位置坐标 (经度, 纬度, 高度)
  - 速度向量 (vx, vy, vz)
  - 加速度向量 (ax, ay, az)
  - IMU角速度 (ωx, ωy, ωz)
- **数学表示**: $\vec{v} = \begin{bmatrix} x \\ y \\ z \end{bmatrix}$

**第14行：3x3矩阵类型别名**

```cpp
using Matrix3d = Eigen::Matrix3d;
```

- **原始类型**: `Eigen::Matrix3d` 是3x3的double矩阵
- **使用场景**:
  - 旋转矩阵 (姿态表示)
  - 协方差矩阵 (不确定性表示)
  - 坐标变换矩阵
- **数学表示**: $M = \begin{bmatrix} m_{11} & m_{12} & m_{13} \\ m_{21} & m_{22} & m_{23} \\ m_{31} & m_{32} & m_{33} \end{bmatrix}$

**第15行：时间戳类型别名**

```cpp
using Timestamp = std::chrono::nanoseconds;
```

- **原始类型**: `std::chrono::nanoseconds` 纳秒精度时间
- **精度选择分析**:
  - **纳秒级 (1e-9秒)**: 足够精确测量微控制器执行时间
  - **VCU控制周期**: 通常1-10ms，纳秒精度绰绰有余
  - **传感器同步**: 高精度时间戳确保数据融合的准确性
- **实际应用**: 
  - 传感器数据时间戳
  - 控制指令发送时间
  - 性能统计和延迟测量

---

## 3. 系统状态枚举

### 第18-22行：系统状态定义

```cpp
// 系统状态枚举
enum class SystemState : uint8_t {
    OFF, STANDBY, READY, RUNNING, 
    DEGRADED, FAULT, EMERGENCY, CALIBRATION
};
```

#### 详细解析

**枚举设计原则**

- **强类型枚举**: 使用 `enum class` 避免隐式转换
- **底层类型**: `uint8_t` 节省内存，适合嵌入式系统
- **状态完整性**: 涵盖系统生命周期的所有关键状态

#### 状态机设计分析

这是一个典型的**有限状态机(FSM)**设计，状态转换关系如下：

```
状态转换图：
    OFF ──────→ STANDBY ──────→ READY ──────→ RUNNING
     ↑            ↓              ↓             ↓
     │            ↓              ↓             ↓
     └────── EMERGENCY ←─────────┴─────────────┘
              ↓
         CALIBRATION
              ↓
        FAULT/DEGRADED
```

#### 各状态详细说明

| 状态            | 数值  | 含义   | 典型场景         | 允许操作         | 退出条件    |
| ------------- | --- | ---- | ------------ | ------------ | ------- |
| `OFF`         | 0   | 系统关闭 | 拖拉机熄火，主电源断开  | 仅允许启动序列      | 接收启动指令  |
| `STANDBY`     | 1   | 待机状态 | 点火开关打开，系统自检  | 系统自检、配置加载    | 自检通过    |
| `READY`       | 2   | 准备就绪 | 发动机启动，等待作业指令 | 接收驾驶员指令、参数设置 | 接收运行指令  |
| `RUNNING`     | 3   | 正常运行 | 田间作业，全功能运行   | 所有控制功能       | 作业完成或故障 |
| `DEGRADED`    | 4   | 降级运行 | 部分传感器故障，功能受限 | 基本控制功能       | 故障修复或加重 |
| `FAULT`       | 5   | 故障状态 | 严重错误，需要干预    | 仅安全相关功能      | 故障清除    |
| `EMERGENCY`   | 6   | 紧急状态 | 紧急停车，立即停止    | 紧急停止程序       | 手动复位    |
| `CALIBRATION` | 7   | 标定状态 | 传感器标定，参数调整   | 标定程序运行       | 标定完成    |

#### 状态转换逻辑示例

```cpp
// 状态转换函数示例
SystemState transitionState(SystemState current, SystemEvent event) {
    switch(current) {
        case SystemState::OFF:
            if(event == SystemEvent::POWER_ON) return SystemState::STANDBY;
            break;

        case SystemState::STANDBY:
            if(event == SystemEvent::SELF_CHECK_PASSED) return SystemState::READY;
            if(event == SystemEvent::EMERGENCY) return SystemState::EMERGENCY;
            break;

        case SystemState::READY:
            if(event == SystemEvent::START_OPERATION) return SystemState::RUNNING;
            if(event == SystemEvent::CALIBRATION_REQUEST) return SystemState::CALIBRATION;
            break;

        case SystemState::RUNNING:
            if(event == SystemEvent::MINOR_FAULT) return SystemState::DEGRADED;
            if(event == SystemEvent::MAJOR_FAULT) return SystemState::FAULT;
            if(event == SystemEvent::EMERGENCY) return SystemState::EMERGENCY;
            break;

        // ... 其他状态转换
    }
    return current; // 无效转换，保持当前状态
}
```

#### 内存和性能考虑

**为什么使用 `uint8_t`？**

1. **内存效率**: 1字节 vs 4字节的默认int，在嵌入式系统中很重要
2. **CAN总线效率**: 状态信息通过CAN总线传输，字节数越少越好
3. **缓存友好**: 更小的数据类型提高缓存命中率
4. **足够的取值范围**: 8位可表示256个状态，远超实际需求

---

## 4. 驱动模式枚举

### 第24-28行：驱动模式定义

```cpp
// 驱动模式枚举
enum class DriveMode : uint8_t {
    ECO, COMFORT, SPORT, 
    PLOWING, SEEDING, TRANSPORT,
    ROAD, FIELD, MANUAL,
    LOADING, UNLOADING, PARKING
};
```

#### 详细解析

这个枚举定义了拖拉机的各种工作模式，体现了**混合动力自主拖拉机**的多样化应用场景。

#### 模式分类分析

**通用驾驶模式 (汽车行业标准)**
| 模式 | 特点 | 动力分配策略 | 适用场景 |
|------|------|-------------|----------|
| `ECO` | 经济模式 | 优先电机驱动，发动机高效区工作 | 轻负载运输，燃油经济性优先 |
| `COMFORT` | 舒适模式 | 平衡动力性和舒适性 | 日常驾驶，平衡各项性能 |
| `SPORT` | 运动模式 | 最大动力输出，快速响应 | 需要强劲动力的场景 |

**农业作业模式 (拖拉机专用)**
| 模式 | 特点 | 控制策略 | 适用场景 |
|------|------|----------|----------|
| `PLOWING` | 犁地模式 | 恒定扭矩输出，深度控制 | 土地翻耕作业 |
| `SEEDING` | 播种模式 | 精确速度控制，均匀播撒 | 种子播种作业 |
| `TRANSPORT` | 运输模式 | 高速巡航，燃油经济 | 田间道路运输 |

**环境适应模式**
| 模式 | 特点 | 适应性调整 | 适用场景 |
|------|------|------------|----------|
| `ROAD` | 道路模式 | 高速行驶优化 | 公路行驶 |
| `FIELD` | 田间模式 | 低速高扭矩 | 田间作业 |
| `MANUAL` | 手动模式 | 驾驶员完全控制 | 特殊操作需求 |

**特殊操作模式**
| 模式 | 特点 | 控制重点 | 适用场景 |
|------|------|----------|----------|
| `LOADING` | 装载模式 | 精确位置控制 | 货物装载 |
| `UNLOADING` | 卸载模式 | 平稳卸载控制 | 货物卸载 |
| `PARKING` | 停车模式 | 最小能耗，安全停放 | 停车状态 |

#### 模式切换逻辑示例

```cpp
// 驱动模式对应的控制参数
struct DriveParameters {
    float maxTorque;        // 最大扭矩限制
    float maxSpeed;         // 最大速度限制
    float responseGain;     // 响应增益
    float efficiencyWeight; // 效率权重
    bool autonomousEnabled; // 是否启用自主功能
};

// 各模式的参数配置
std::map<DriveMode, DriveParameters> modeParameters = {
    {DriveMode::ECO, {0.7f, 0.8f, 0.5f, 1.0f, true}},
    {DriveMode::COMFORT, {0.8f, 0.9f, 0.7f, 0.8f, true}},
    {DriveMode::SPORT, {1.0f, 1.0f, 1.0f, 0.5f, false}},
    {DriveMode::PLOWING, {1.0f, 0.3f, 0.8f, 0.7f, true}},
    {DriveMode::SEEDING, {0.6f, 0.4f, 0.9f, 0.9f, true}},
    // ... 其他模式配置
};
```

---

## 5. CVT制造商枚举

### 第30-35行：CVT制造商定义

```cpp
// CVT制造商枚举
enum class CVTManufacturer {
    UNKNOWN, JOHN_DEERE, CASE_IH, CLAAS, 
    AGCO_FENDT, AGCO_MASSEY_FERGUSON, 
    NEW_HOLLAND, KUBOTA, CNH, DEUTZ_FAHR, SAME,
    VALTRA, ZETOR, YANMAR
};
```

#### 详细解析

这个枚举体现了VCU系统的**通用性设计**，支持多个主流农机制造商的CVT系统。

#### 制造商分析

**全球农机巨头**
| 制造商 | 总部 | CVT技术特点 | 市场地位 |
|--------|------|-------------|----------|
| `JOHN_DEERE` | 美国 | AutoPowr CVT，成熟可靠 | 全球第一 |
| `CASE_IH` | 美国 | CVX系列，高效传动 | 北美领先 |
| `NEW_HOLLAND` | 意大利 | VT系列，操控性好 | 欧洲强势 |

**欧洲专业制造商**
| 制造商 | 总部 | CVT技术特点 | 市场地位 |
|--------|------|-------------|----------|
| `CLAAS` | 德国 | CMATIC系列，精密控制 | 收获机械领先 |
| `AGCO_FENDT` | 德国 | Vario系列，技术先进 | 高端市场 |
| `DEUTZ_FAHR` | 德国 | TTV系列，可靠耐用 | 传统强势 |

**亚洲制造商**
| 制造商 | 总部 | CVT技术特点 | 市场地位 |
|--------|------|-------------|----------|
| `KUBOTA` | 日本 | 小型化CVT，精密制造 | 小型农机领先 |
| `YANMAR` | 日本 | 发动机+CVT集成 | 动力系统专家 |

#### 设计意义

**为什么需要制造商枚举？**

1. **协议适配**: 不同制造商的CVT通信协议可能不同
   
   ```cpp
   // 根据制造商选择通信协议
   CANProtocol selectCVTProtocol(CVTManufacturer manufacturer) {
    switch(manufacturer) {
        case CVTManufacturer::JOHN_DEERE:
            return CANProtocol::J1939_DEERE;
        case CVTManufacturer::CLAAS:
            return CANProtocol::J1939_CLAAS;
        default:
            return CANProtocol::J1939_STANDARD;
    }
   }
   ```

2. **参数优化**: 不同CVT的控制参数需要针对性调整
   
   ```cpp
   // CVT控制参数根据制造商调整
   CVTControlParams getCVTParams(CVTManufacturer manufacturer) {
    switch(manufacturer) {
        case CVTManufacturer::JOHN_DEERE:
            return {0.5f, 3.5f, 0.1f, 0.15f, 0.02f, 0.8f, 0.6f, 0.7f};
        case CVTManufacturer::FENDT:
            return {0.6f, 4.0f, 0.12f, 0.18f, 0.025f, 0.85f, 0.65f, 0.75f};
        // ... 其他制造商参数
    }
   }
   ```

3. **故障诊断**: 不同CVT的故障代码和诊断逻辑不同

4. **性能优化**: 针对不同CVT特性进行算法优化

---

## 6. 负载变化类型枚举

### 第37-43行：负载变化类型定义

```cpp
// 负载变化类型
enum class LoadChangeType {
    NO_CHANGE, TORQUE_INCREASE, TORQUE_DECREASE,
    LOAD_SPIKE, LOAD_DROP, ENGINE_DERATE, OVERLOAD,
    WHEEL_SLIP, IMPLEMENT_BLOCKAGE, TERRAIN_CHANGE,
    SOIL_VARIATION, IMPLEMENT_ENGAGEMENT, IMPLEMENT_DISENGAGEMENT
};
```

#### 详细解析

这个枚举定义了拖拉机在田间作业时可能遇到的各种负载变化情况，是**智能负载管理**的基础。

#### 负载变化分类

**基本负载变化**
| 类型 | 特征 | 典型原因 | VCU响应策略 |
|------|------|----------|-------------|
| `NO_CHANGE` | 负载稳定 | 均匀土壤，稳定作业 | 维持当前控制参数 |
| `TORQUE_INCREASE` | 扭矩需求增加 | 土壤变硬，坡度增加 | 增加发动机输出或电机辅助 |
| `TORQUE_DECREASE` | 扭矩需求减少 | 土壤变软，下坡 | 减少动力输出，考虑能量回收 |

**突发负载变化**
| 类型 | 特征 | 典型原因 | VCU响应策略 |
|------|------|----------|-------------|
| `LOAD_SPIKE` | 负载突然增加 | 遇到石头，犁具卡住 | 快速增加扭矩，防止熄火 |
| `LOAD_DROP` | 负载突然减少 | 犁具脱离土壤 | 快速减少扭矩，防止超速 |
| `OVERLOAD` | 负载超过限制 | 土壤过硬，农具过载 | 降低速度或提升农具 |

**系统相关变化**
| 类型 | 特征 | 典型原因 | VCU响应策略 |
|------|------|----------|-------------|
| `ENGINE_DERATE` | 发动机降功率 | 过热，排放限制 | 电机补偿，降低作业强度 |
| `WHEEL_SLIP` | 车轮打滑 | 土壤湿滑，扭矩过大 | 减少扭矩，调整传动比 |

**农具相关变化**
| 类型 | 特征 | 典型原因 | VCU响应策略 |
|------|------|----------|-------------|
| `IMPLEMENT_BLOCKAGE` | 农具堵塞 | 作物残茬，杂草缠绕 | 停止前进，清理程序 |
| `IMPLEMENT_ENGAGEMENT` | 农具入土 | 开始作业，犁具下降 | 逐渐增加负载，平稳过渡 |
| `IMPLEMENT_DISENGAGEMENT` | 农具出土 | 作业结束，犁具提升 | 逐渐减少负载，准备转移 |

**环境相关变化**
| 类型 | 特征 | 典型原因 | VCU响应策略 |
|------|------|----------|-------------|
| `TERRAIN_CHANGE` | 地形变化 | 坡度变化，地面起伏 | 调整传动比，优化牵引力 |
| `SOIL_VARIATION` | 土壤变化 | 土质差异，湿度变化 | 调整作业参数，优化效果 |

#### 负载检测算法示例

```cpp
// 负载变化检测算法
class LoadChangeDetector {
private:
    std::vector<float> loadHistory;
    float threshold_spike = 0.3f;    // 突增阈值 (30%)
    float threshold_drop = 0.25f;    // 突降阈值 (25%)

public:
    LoadChangeType detectLoadChange(float currentLoad) {
        if(loadHistory.size() < 5) {
            loadHistory.push_back(currentLoad);
            return LoadChangeType::NO_CHANGE;
        }

        float avgLoad = calculateAverage(loadHistory);
        float loadChange = (currentLoad - avgLoad) / avgLoad;

        // 突发变化检测
        if(loadChange > threshold_spike) {
            return LoadChangeType::LOAD_SPIKE;
        } else if(loadChange < -threshold_drop) {
            return LoadChangeType::LOAD_DROP;
        }

        // 趋势变化检测
        float trend = calculateTrend(loadHistory);
        if(trend > 0.1f) {
            return LoadChangeType::TORQUE_INCREASE;
        } else if(trend < -0.1f) {
            return LoadChangeType::TORQUE_DECREASE;
        }

        return LoadChangeType::NO_CHANGE;
    }
};
```

#### 应用场景示例

**犁地作业中的负载管理**

```cpp
void handlePlowingLoadChange(LoadChangeType changeType) {
    switch(changeType) {
        case LoadChangeType::SOIL_VARIATION:
            // 土壤变化：调整犁深和速度
            adjustPlowDepth(soilHardness);
            adjustSpeed(optimalSpeed);
            break;

        case LoadChangeType::IMPLEMENT_BLOCKAGE:
            // 犁具堵塞：停止前进，启动清理程序
            stopForward();
            activateCleaningSequence();
            break;

        case LoadChangeType::WHEEL_SLIP:
            // 车轮打滑：减少扭矩，调整传动比
            reduceTorque(0.8f);
            adjustCVTRatio(lowerRatio);
            break;
    }
}
```

这个枚举的设计体现了VCU系统对**复杂农业环境**的深度理解和**智能适应能力**。

---

## 7. 故障相关枚举

### 第45-53行：负载趋势和故障等级定义

```cpp
// 负载趋势
enum class LoadTrend {
    STEADY, INCREASING, DECREASING, 
    OSCILLATING, STEP_CHANGE, RANDOM,
    CYCLIC, EXPONENTIAL, LOGARITHMIC
};

// 故障严重等级
enum class FaultSeverity : uint8_t {
    NONE, INFO, WARNING, ERROR, CRITICAL, FATAL
};

// 控制模式
enum class ControlMode : uint8_t {
    AUTO, MANUAL, SEMI_AUTO, DEGRADED, SAFETY
};
```

#### 7.1 负载趋势枚举 (LoadTrend)

**设计目的**: 描述负载随时间的变化模式，用于**预测性控制**和**趋势分析**。

| 趋势类型          | 数学特征                               | 物理含义    | 控制策略      |
| ------------- | ---------------------------------- | ------- | --------- |
| `STEADY`      | $\frac{dL}{dt} \approx 0$          | 负载稳定不变  | 维持当前参数    |
| `INCREASING`  | $\frac{dL}{dt} > 0$                | 负载持续增加  | 预测性增加动力   |
| `DECREASING`  | $\frac{dL}{dt} < 0$                | 负载持续减少  | 预测性减少动力   |
| `OSCILLATING` | $L(t) = A\sin(\omega t)$           | 负载周期性波动 | 滤波处理，平均控制 |
| `STEP_CHANGE` | $L(t) = L_0 + \Delta L \cdot u(t)$ | 负载阶跃变化  | 快速响应控制    |
| `RANDOM`      | 随机过程                               | 负载随机变化  | 鲁棒控制策略    |
| `CYCLIC`      | $L(t+T) = L(t)$                    | 负载周期性重复 | 学习型控制     |
| `EXPONENTIAL` | $L(t) = L_0 e^{at}$                | 负载指数变化  | 非线性预测控制   |
| `LOGARITHMIC` | $L(t) = a\log(t) + b$              | 负载对数变化  | 渐进式调整     |

**趋势检测算法示例**:

```cpp
class LoadTrendAnalyzer {
private:
    std::deque<float> loadBuffer;
    static constexpr size_t BUFFER_SIZE = 50;

public:
    LoadTrend analyzeTrend(float currentLoad) {
        loadBuffer.push_back(currentLoad);
        if(loadBuffer.size() > BUFFER_SIZE) {
            loadBuffer.pop_front();
        }

        if(loadBuffer.size() < 10) return LoadTrend::STEADY;

        // 线性回归检测趋势
        auto [slope, r_squared] = linearRegression(loadBuffer);

        if(r_squared > 0.8) {  // 强线性相关
            if(slope > 0.01) return LoadTrend::INCREASING;
            if(slope < -0.01) return LoadTrend::DECREASING;
        }

        // 周期性检测
        if(detectPeriodicity(loadBuffer)) {
            return LoadTrend::CYCLIC;
        }

        // 振荡检测
        if(detectOscillation(loadBuffer)) {
            return LoadTrend::OSCILLATING;
        }

        return LoadTrend::STEADY;
    }
};
```

#### 7.2 故障严重等级枚举 (FaultSeverity)

**设计目的**: 建立**分级故障管理**体系，确保系统安全性和可用性。

| 等级         | 数值  | 含义   | 典型故障   | 系统响应    | 操作员动作 |
| ---------- | --- | ---- | ------ | ------- | ----- |
| `NONE`     | 0   | 无故障  | 系统正常   | 正常运行    | 无需动作  |
| `INFO`     | 1   | 信息提示 | 维护提醒   | 记录日志    | 注意信息  |
| `WARNING`  | 2   | 警告   | 传感器漂移  | 继续运行，监控 | 计划检查  |
| `ERROR`    | 3   | 错误   | 通信超时   | 降级运行    | 及时处理  |
| `CRITICAL` | 4   | 严重故障 | 过热保护   | 安全停机    | 立即处理  |
| `FATAL`    | 5   | 致命故障 | 安全系统失效 | 紧急停机    | 紧急维修  |

**故障处理策略**:

```cpp
class FaultManager {
public:
    void handleFault(uint16_t faultCode, FaultSeverity severity) {
        switch(severity) {
            case FaultSeverity::INFO:
                logInfo(faultCode);
                break;

            case FaultSeverity::WARNING:
                logWarning(faultCode);
                activateMonitoring(faultCode);
                break;

            case FaultSeverity::ERROR:
                logError(faultCode);
                enterDegradedMode();
                notifyOperator(faultCode);
                break;

            case FaultSeverity::CRITICAL:
                logCritical(faultCode);
                initiateSafeShutdown();
                alertOperator(faultCode);
                break;

            case FaultSeverity::FATAL:
                logFatal(faultCode);
                emergencyStop();
                emergencyAlert(faultCode);
                break;
        }
    }
};
```

#### 7.3 控制模式枚举 (ControlMode)

**设计目的**: 定义VCU的**控制权限级别**，平衡自动化和人工干预。

| 模式          | 自动化程度 | 人工干预 | 适用场景      | 安全级别 |
| ----------- | ----- | ---- | --------- | ---- |
| `AUTO`      | 100%  | 最小   | 标准田间作业    | 高    |
| `MANUAL`    | 0%    | 完全   | 特殊操作，故障处理 | 中    |
| `SEMI_AUTO` | 70%   | 部分   | 复杂地形，学习阶段 | 高    |
| `DEGRADED`  | 50%   | 较多   | 传感器故障时    | 中    |
| `SAFETY`    | 安全功能  | 受限   | 紧急情况      | 最高   |

**模式切换逻辑**:

```cpp
class ControlModeManager {
private:
    ControlMode currentMode = ControlMode::AUTO;

public:
    bool switchMode(ControlMode newMode, SystemHealthStatus health) {
        // 安全检查
        if(!isSafeToSwitch(currentMode, newMode, health)) {
            return false;
        }

        switch(newMode) {
            case ControlMode::AUTO:
                if(health.overallHealth > 0.9f) {
                    enableFullAutonomy();
                    currentMode = newMode;
                    return true;
                }
                break;

            case ControlMode::DEGRADED:
                disableNonEssentialFunctions();
                enableBasicControl();
                currentMode = newMode;
                return true;

            case ControlMode::SAFETY:
                activateEmergencyProtocol();
                currentMode = newMode;
                return true;
        }
        return false;
    }
};
```

这些枚举类型构成了VCU系统的**状态管理框架**，确保系统在各种情况下都能安全、可靠地运行。
