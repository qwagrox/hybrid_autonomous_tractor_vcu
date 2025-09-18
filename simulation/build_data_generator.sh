#!/bin/bash

# VCU动力总成数据生成器编译脚本

echo "🔨 编译VCU动力总成数据生成器..."

# 检查编译器
if ! command -v g++ &> /dev/null; then
    echo "❌ 错误: 未找到g++编译器"
    echo "请安装: sudo apt-get install build-essential"
    exit 1
fi

# 编译数据生成器
echo "正在编译 powertrain_data_generator.cpp..."
g++ -std=c++17 -O2 -Wall -Wextra \
    powertrain_data_generator.cpp \
    -o powertrain_data_generator \
    -lm

if [ $? -eq 0 ]; then
    echo "✅ 编译成功！"
    echo ""
    
    # 显示使用方法
    echo "📋 使用方法:"
    echo "./powertrain_data_generator [场景] [时长秒] [采样间隔秒]"
    echo ""
    echo "🎯 可用场景:"
    echo "  normal_operation    - 正常作业场景"
    echo "  high_load_plowing   - 高负载犁地场景"
    echo "  hybrid_mode_test    - 混合动力模式测试"
    echo "  battery_charge_test - 电池充电测试"
    echo ""
    echo "💡 示例:"
    echo "./powertrain_data_generator normal_operation 180 0.1"
    echo "./powertrain_data_generator high_load_plowing 300 0.05"
    echo "./powertrain_data_generator hybrid_mode_test 240 0.1"
    echo ""
    
    # 运行快速测试
    echo "🚀 运行快速测试 (30秒正常作业场景)..."
    ./powertrain_data_generator normal_operation 30 0.1
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "🎉 数据生成器测试成功！"
        echo "📁 生成的文件: powertrain_data_normal_operation.csv"
        
        # 显示文件信息
        if [ -f "powertrain_data_normal_operation.csv" ]; then
            lines=$(wc -l < powertrain_data_normal_operation.csv)
            size=$(du -h powertrain_data_normal_operation.csv | cut -f1)
            echo "📊 文件信息: $lines 行, $size"
            
            echo ""
            echo "📋 数据预览 (前5行):"
            head -5 powertrain_data_normal_operation.csv
        fi
    else
        echo "❌ 数据生成器测试失败"
    fi
    
else
    echo "❌ 编译失败！"
    echo "请检查代码错误或依赖项"
    exit 1
fi
