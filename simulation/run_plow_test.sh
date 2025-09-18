#!/bin/bash

# VCU犁地阻力测试仿真编译和运行脚本

echo "=== VCU犁地阻力智能决策仿真测试 ==="
echo "编译和运行脚本 v1.0"
echo ""

# 检查编译器
if ! command -v g++ &> /dev/null; then
    echo "❌ 错误: 未找到g++编译器"
    echo "请安装: sudo apt-get install build-essential"
    exit 1
fi

# 检查Python
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误: 未找到Python3"
    echo "请安装: sudo apt-get install python3"
    exit 1
fi

# 创建输出目录
mkdir -p results
mkdir -p logs

echo "🔨 编译仿真程序..."

# 编译仿真程序
g++ -std=c++17 -O2 -Wall -Wextra \
    plow_resistance_simulation.cpp \
    -o plow_resistance_simulation \
    -lm

if [ $? -ne 0 ]; then
    echo "❌ 编译失败！"
    exit 1
fi

echo "✅ 编译成功！"
echo ""

echo "🚀 运行仿真测试..."
echo "测试时长: 180秒 (3分钟)"
echo "仿真步长: 10ms"
echo ""

# 运行仿真
./plow_resistance_simulation

simulation_result=$?

echo ""
echo "📊 仿真数据分析..."

# 检查是否有matplotlib
python3 -c "import matplotlib" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ 开始数据分析和可视化..."
    
    # 运行数据分析
    python3 analyze_test_results.py
    
    if [ $? -eq 0 ]; then
        echo "✅ 数据分析完成！"
        
        # 移动结果文件到results目录
        if [ -f "plow_resistance_simulation_data.csv" ]; then
            mv plow_resistance_simulation_data.csv results/
            echo "📁 数据文件已移动到: results/plow_resistance_simulation_data.csv"
        fi
        
        if [ -f "plow_resistance_test_analysis.png" ]; then
            mv plow_resistance_test_analysis.png results/
            echo "📊 分析图表已保存到: results/plow_resistance_test_analysis.png"
        fi
        
        if [ -f "plow_resistance_test_report.json" ]; then
            mv plow_resistance_test_report.json results/
            echo "📋 分析报告已保存到: results/plow_resistance_test_report.json"
        fi
        
    else
        echo "⚠️ 数据分析失败，但仿真数据已保存"
    fi
else
    echo "⚠️ 未安装matplotlib，跳过可视化分析"
    echo "安装命令: pip3 install matplotlib pandas numpy"
    
    # 仍然移动数据文件
    if [ -f "plow_resistance_simulation_data.csv" ]; then
        mv plow_resistance_simulation_data.csv results/
        echo "📁 数据文件已保存到: results/plow_resistance_simulation_data.csv"
    fi
fi

echo ""
echo "=== 测试完成 ==="

if [ $simulation_result -eq 0 ]; then
    echo "🎉 测试结果: ✅ 通过"
    echo ""
    echo "📁 输出文件:"
    echo "  - results/plow_resistance_simulation_data.csv (仿真数据)"
    if [ -f "results/plow_resistance_test_analysis.png" ]; then
        echo "  - results/plow_resistance_test_analysis.png (可视化图表)"
    fi
    if [ -f "results/plow_resistance_test_report.json" ]; then
        echo "  - results/plow_resistance_test_report.json (分析报告)"
    fi
    
    echo ""
    echo "🔍 建议后续操作:"
    echo "  1. 查看可视化图表了解系统响应过程"
    echo "  2. 分析JSON报告中的详细性能指标"
    echo "  3. 根据建议优化VCU控制算法"
    echo "  4. 调整测试参数进行更多场景验证"
    
else
    echo "❌ 测试结果: 失败"
    echo ""
    echo "🔧 故障排除建议:"
    echo "  1. 检查仿真日志中的错误信息"
    echo "  2. 验证VCU控制算法参数"
    echo "  3. 调整异常检测阈值"
    echo "  4. 优化多目标优化权重"
fi

echo ""
echo "📖 更多信息请参考:"
echo "  - docs/simulation/plow_resistance_test_detailed_design.md"
echo "  - docs/simulation/test_cases_and_validation.md"

exit $simulation_result
