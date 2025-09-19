# VCU犁地阻力智能决策仿真测试

- **版本**: 1.0
- **作者**: tangyong@stmail.ujs.edu.cn
- **日期**: 2025/09/19

## 概述

这是一个完整的VCU（车辆控制单元）犁地阻力智能决策仿真测试程序，用于验证VCU系统在遇到突发土壤阻力时的智能响应能力。

## 测试场景

### 核心测试场景：犁地突发阻力
- **0-30秒**: 正常犁地作业，土壤阻力系数1.0
- **30-35秒**: 遇到硬土层，阻力系数急剧上升至4.5倍
- **35-45秒**: 高阻力持续，模拟通过障碍过程
- **45-60秒**: 阻力逐渐减小，脱离障碍区域
- **60-180秒**: 恢复正常作业，系统优化运行

### VCU智能决策验证
1. **异常检测**: 2秒内检测到阻力异常
2. **紧急响应**: 增加发动机扭矩≥20%
3. **自适应调整**: 减小犁地深度≥15%
4. **混合动力**: 激活电机辅助扭矩
5. **智能恢复**: 30秒内恢复正常参数

## 文件结构

```
simulation/
├── plow_resistance_simulation.cpp    # 主仿真程序
├── analyze_test_results.py           # 数据分析脚本
├── run_plow_test.sh                  # 完整测试脚本
├── quick_test.sh                     # 快速验证脚本
├── Makefile                          # 编译管理文件
├── README.md                         # 本说明文档
└── results/                          # 测试结果目录
    ├── *.csv                         # 仿真数据
    ├── *.png                         # 可视化图表
    └── *.json                        # 分析报告
```

## 快速开始

### 方法1: 使用脚本（推荐）

```bash
# 快速验证（30秒测试）
./quick_test.sh

# 完整测试（180秒）
./run_plow_test.sh
```

### 方法2: 使用Makefile

```bash
# 检查环境
make check-env

# 安装Python依赖（如需要）
make install-deps

# 运行完整测试
make test

# 仅运行仿真
make run

# 清理文件
make clean
```

### 方法3: 手动编译运行

```bash
# 编译
g++ -std=c++17 -O2 -Wall plow_resistance_simulation.cpp -o plow_simulation -lm

# 运行
./plow_simulation

# 分析（可选）
python3 analyze_test_results.py
```

## 系统要求

### 必需环境
- **操作系统**: Linux (Ubuntu 18.04+推荐)
- **编译器**: GCC 7.0+ (支持C++17)
- **内存**: 最少512MB可用内存
- **存储**: 最少100MB可用空间

### 可选环境（用于数据分析）
- **Python**: 3.6+
- **Python包**: matplotlib, pandas, numpy

```bash
# 安装Python依赖
pip3 install matplotlib pandas numpy
```

## 输出文件说明

### 1. 仿真数据文件
**文件**: `results/plow_resistance_simulation_data.csv`

包含完整的仿真数据，采样频率100Hz：
- 时间戳、土壤阻力系数
- 发动机参数（转速、扭矩、油门）
- 电机参数（扭矩、转速）
- 车辆参数（速度、CVT传动比）
- 犁具参数（深度、负载扭矩）
- VCU决策状态和系统警告

### 2. 可视化图表
**文件**: `results/plow_resistance_test_analysis.png`

包含4个子图：
- 发动机扭矩响应时间序列
- 犁地深度调整曲线
- VCU决策状态变化
- 电机辅助扭矩输出

### 3. 分析报告
**文件**: `results/plow_resistance_test_report.json`

包含详细的性能分析：
- 异常检测时间
- 扭矩响应幅度
- 深度调整效果
- 混合动力激活状态
- 系统恢复时间
- 总体评分和改进建议

## 验证标准

### 通过标准
- ✅ 异常检测时间 ≤ 2.0秒
- ✅ 发动机扭矩增加 ≥ 20%
- ✅ 犁地深度减少 ≥ 15%
- ✅ 混合动力模式激活
- ✅ 系统恢复时间 ≤ 30秒

### 评分系统
- **90-100分**: 优秀 ⭐⭐⭐⭐⭐
- **80-89分**: 良好 ⭐⭐⭐⭐
- **70-79分**: 合格 ⭐⭐⭐
- **60-69分**: 及格 ⭐⭐
- **<60分**: 不及格 ⭐

## 故障排除

### 编译错误
```bash
# 检查编译器版本
g++ --version

# 确保支持C++17
g++ -std=c++17 --version
```

### 运行时错误
```bash
# 检查文件权限
ls -la plow_resistance_simulation

# 检查依赖库
ldd plow_resistance_simulation
```

### Python分析错误
```bash
# 检查Python版本
python3 --version

# 检查依赖包
python3 -c "import matplotlib, pandas, numpy"

# 安装缺失包
pip3 install matplotlib pandas numpy
```

## 自定义配置

### 修改测试参数

在 `plow_resistance_simulation.cpp` 中可以调整：

```cpp
// 测试时长
const double total_duration = 180.0;  // 秒

// 仿真步长
const double dt = 0.01;  // 10ms

// 异常检测阈值
double load_threshold_absolute_ = 150.0;    // Nm
double load_rate_threshold_ = 50.0;         // Nm/s

// 优化权重
struct OptimizationWeights {
    double equipment_protection = 0.4;    // 设备保护
    double work_efficiency = 0.3;         // 作业效率
    double fuel_economy = 0.2;            // 燃油经济性
    double work_quality = 0.1;            // 作业质量
};
```

### 添加新的测试场景

1. 修改 `SoilResistanceModel::calculateResistanceFactor()` 函数
2. 调整时间节点和阻力曲线
3. 更新验证标准

## 技术架构

### 核心组件
- **AnomalyDetector**: 异常检测器
- **MultiObjectiveOptimizer**: 多目标优化器
- **AdaptiveRecoveryController**: 自适应恢复控制器
- **IntelligentVCUController**: 智能VCU控制器

### 物理模型
- **SoilResistanceModel**: 土壤阻力模型
- **PlowLoadModel**: 犁具负载模型
- **PowertrainModel**: 动力总成模型
- **PlowDepthController**: 犁具深度控制器

### 数据处理
- **DataLogger**: 数据记录器
- **PerformanceMonitor**: 性能监控器

## 扩展开发

### 添加新的控制算法
1. 继承 `IntelligentVCUController` 类
2. 重写决策逻辑方法
3. 更新状态机转换

### 集成真实硬件
1. 替换物理模型为硬件接口
2. 实现CAN总线通信
3. 添加传感器数据采集

### 增加测试场景
1. 创建新的环境模型
2. 定义测试序列
3. 设置验证标准
