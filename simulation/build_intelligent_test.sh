#!/bin/bash

# VCU智能决策测试编译脚本

echo "=== 编译VCU智能决策测试程序 ==="

# 检查编译器
if ! command -v g++ &> /dev/null; then
    echo "错误: 未找到g++编译器"
    exit 1
fi

# 创建构建目录
mkdir -p build
cd build

# 编译智能决策测试程序
echo "正在编译智能决策测试..."
g++ -std=c++17 -O2 -Wall -Wextra \
    -I../models \
    -I.. \
    ../intelligent_decision_test.cpp \
    ../models/powertrain_model.cpp \
    ../models/implement_model.cpp \
    -o intelligent_decision_test

if [ $? -eq 0 ]; then
    echo "✅ 编译成功!"
    echo "可执行文件: build/intelligent_decision_test"
    
    # 运行测试
    echo ""
    echo "=== 运行智能决策测试 ==="
    ./intelligent_decision_test
    
    echo ""
    echo "=== 测试完成 ==="
    echo "查看生成的文件:"
    echo "  - vcu_decision_log.csv (决策日志)"
    echo "  - intelligent_decision_test_report.html (测试报告)"
    
else
    echo "❌ 编译失败!"
    exit 1
fi
