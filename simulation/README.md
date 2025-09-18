# 仿真子系统如何运行测试

- **版本**: 1.0
- **作者**: tangyong@agrox.cloud 
- **日期**: 2025/09/18

## 1. 编译测试程序

```
cd simulation
./build_intelligent_test.sh
```

## 2. 查看实时输出

[VCU] 检测到异常阻力! 负载: 180.5Nm, 变化率: 65.2Nm/s
[VCU] 确认高阻力情况，启动紧急响应  
[VCU] 启用混合动力模式，电机辅助扭矩: 80Nm
[VCU] 阻力减小，准备进入恢复模式
[VCU] 完全恢复正常作业状态

✅ 异常检测时间: 1.2s
✅ 最大发动机扭矩: 312Nm  
✅ 犁地深度减少: 18.5%
✅ 系统恢复时间: 87.3s

## 3. 分析测试数据

- `vcu_decision_log.csv`: 详细的决策过程数据
- `intelligent_decision_test_report.html`: 可视化测试报告
