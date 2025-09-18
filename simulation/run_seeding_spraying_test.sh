#!/bin/bash

# 播种和喷药关键测试用例运行脚本

echo "🌱 播种和喷药关键测试用例仿真系统"
echo "=================================="

# 检查编译器
if ! command -v g++ &> /dev/null; then
    echo "❌ 错误: 未找到g++编译器"
    echo "请安装: sudo apt-get install build-essential"
    exit 1
fi

# 创建结果目录
mkdir -p results/seeding_spraying_tests
mkdir -p logs

# 编译仿真程序
echo "🔨 编译播种喷药测试仿真程序..."
g++ -std=c++17 -O2 -Wall -Wextra \
    seeding_spraying_simulation.cpp \
    -o seeding_spraying_test \
    -lm

if [ $? -ne 0 ]; then
    echo "❌ 编译失败！"
    exit 1
fi

echo "✅ 编译成功！"
echo ""

# 显示可用测试用例
echo "📋 可用测试用例:"
echo "  seeding_depth      - 精准播种深度控制测试"
echo "  variable_seeding   - 变量播种率控制测试"
echo "  precision_spraying - 精准喷药压力流量控制测试"
echo "  variable_spraying  - 变量喷药控制测试"
echo "  combined_operation - 播种喷药联合作业测试"
echo ""

# 检查命令行参数
if [ $# -eq 0 ]; then
    echo "🚀 运行所有测试用例..."
    ./seeding_spraying_test > logs/seeding_spraying_test_log.txt 2>&1
else
    echo "🚀 运行指定测试用例: $@"
    ./seeding_spraying_test "$@" > logs/seeding_spraying_test_log.txt 2>&1
fi

# 检查运行结果
if [ $? -eq 0 ]; then
    echo "✅ 测试执行成功！"
    
    # 移动结果文件
    mv *.csv results/seeding_spraying_tests/ 2>/dev/null
    mv *.txt results/seeding_spraying_tests/ 2>/dev/null
    
    # 显示结果摘要
    echo ""
    echo "📊 测试结果摘要:"
    echo "----------------------------------------"
    
    # 从日志文件提取关键信息
    if [ -f "logs/seeding_spraying_test_log.txt" ]; then
        grep -E "(✅|❌|通过率|深度控制精度|播种率变化|喷药精度)" logs/seeding_spraying_test_log.txt
    fi
    
    echo ""
    echo "📁 输出文件:"
    ls -la results/seeding_spraying_tests/
    
    echo ""
    echo "📋 使用建议:"
    echo "1. 查看详细日志: cat logs/seeding_spraying_test_log.txt"
    echo "2. 分析CSV数据: python3 analyze_seeding_spraying_data.py"
    echo "3. 生成可视化图表: python3 visualize_seeding_spraying.py"
    
else
    echo "❌ 测试执行失败！"
    echo "查看错误日志: cat logs/seeding_spraying_test_log.txt"
    exit 1
fi

echo ""
echo "🎉 播种喷药测试完成！"
