#!/bin/bash

# VCU犁地阻力测试快速验证脚本
# 用于快速验证仿真程序是否正常工作

echo "🚀 VCU犁地阻力测试 - 快速验证"
echo "================================"

# 检查文件是否存在
if [ ! -f "plow_resistance_simulation.cpp" ]; then
    echo "❌ 错误: 未找到仿真程序源文件"
    exit 1
fi

if [ ! -f "analyze_test_results.py" ]; then
    echo "❌ 错误: 未找到数据分析脚本"
    exit 1
fi

# 快速编译测试
echo "🔨 快速编译测试..."
g++ -std=c++17 -O0 -Wall plow_resistance_simulation.cpp -o test_simulation -lm

if [ $? -ne 0 ]; then
    echo "❌ 编译失败！"
    exit 1
fi

echo "✅ 编译成功"

# 运行短时间测试（30秒）
echo "⚡ 运行30秒快速测试..."

# 修改源码中的测试时长为30秒（临时）
sed 's/const double total_duration = 180.0/const double total_duration = 30.0/' \
    plow_resistance_simulation.cpp > temp_simulation.cpp

g++ -std=c++17 -O0 -Wall temp_simulation.cpp -o quick_test_simulation -lm

if [ $? -eq 0 ]; then
    echo "开始快速仿真..."
    ./quick_test_simulation
    
    test_result=$?
    
    if [ $test_result -eq 0 ]; then
        echo "✅ 快速测试通过！"
        echo ""
        echo "📊 检查输出文件..."
        
        if [ -f "plow_resistance_simulation_data.csv" ]; then
            lines=$(wc -l < plow_resistance_simulation_data.csv)
            echo "✅ 数据文件生成成功，包含 $lines 行数据"
            
            # 显示前几行数据
            echo ""
            echo "📋 数据样本（前5行）:"
            head -5 plow_resistance_simulation_data.csv
            
            # 清理临时文件
            rm -f plow_resistance_simulation_data.csv
        else
            echo "⚠️ 未生成数据文件"
        fi
        
        echo ""
        echo "🎉 快速验证完成！系统工作正常。"
        echo "💡 运行完整测试: make test 或 ./run_plow_test.sh"
        
    else
        echo "❌ 快速测试失败"
    fi
else
    echo "❌ 快速测试编译失败"
fi

# 清理临时文件
rm -f test_simulation quick_test_simulation temp_simulation.cpp

exit $test_result
