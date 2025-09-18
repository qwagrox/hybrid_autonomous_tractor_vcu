# VCU犁地阻力仿真测试 Makefile

CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -g
LDFLAGS = -lm

# 目标文件
TARGET = plow_resistance_simulation
SOURCES = plow_resistance_simulation.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# 输出目录
RESULTS_DIR = results
LOGS_DIR = logs

# 默认目标
all: $(TARGET)

# 编译主程序
$(TARGET): $(OBJECTS)
	@echo "🔗 链接程序..."
	$(CXX) $(OBJECTS) -o $(TARGET) $(LDFLAGS)
	@echo "✅ 编译完成: $(TARGET)"

# 编译对象文件
%.o: %.cpp
	@echo "🔨 编译: $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 创建目录
dirs:
	@mkdir -p $(RESULTS_DIR)
	@mkdir -p $(LOGS_DIR)

# 运行仿真测试
run: $(TARGET) dirs
	@echo "🚀 运行犁地阻力智能决策仿真测试..."
	@echo "测试时长: 180秒"
	@echo "仿真步长: 10ms"
	@echo ""
	./$(TARGET)

# 运行完整测试（包含数据分析）
test: run
	@echo ""
	@echo "📊 开始数据分析..."
	@if command -v python3 >/dev/null 2>&1; then \
		if python3 -c "import matplotlib, pandas, numpy" 2>/dev/null; then \
			python3 analyze_test_results.py; \
			echo "✅ 数据分析完成"; \
		else \
			echo "⚠️ 缺少Python依赖包，请安装: pip3 install matplotlib pandas numpy"; \
		fi; \
	else \
		echo "⚠️ 未找到Python3"; \
	fi
	@$(MAKE) organize

# 整理输出文件
organize:
	@echo "📁 整理输出文件..."
	@if [ -f "plow_resistance_simulation_data.csv" ]; then \
		mv plow_resistance_simulation_data.csv $(RESULTS_DIR)/; \
		echo "  - 数据文件: $(RESULTS_DIR)/plow_resistance_simulation_data.csv"; \
	fi
	@if [ -f "plow_resistance_test_analysis.png" ]; then \
		mv plow_resistance_test_analysis.png $(RESULTS_DIR)/; \
		echo "  - 分析图表: $(RESULTS_DIR)/plow_resistance_test_analysis.png"; \
	fi
	@if [ -f "plow_resistance_test_report.json" ]; then \
		mv plow_resistance_test_report.json $(RESULTS_DIR)/; \
		echo "  - 分析报告: $(RESULTS_DIR)/plow_resistance_test_report.json"; \
	fi

# 清理编译文件
clean:
	@echo "🧹 清理编译文件..."
	rm -f $(OBJECTS) $(TARGET)
	@echo "✅ 清理完成"

# 清理所有文件（包括结果）
clean-all: clean
	@echo "🧹 清理所有文件..."
	rm -rf $(RESULTS_DIR) $(LOGS_DIR)
	@echo "✅ 全部清理完成"

# 安装Python依赖
install-deps:
	@echo "📦 安装Python依赖包..."
	pip3 install matplotlib pandas numpy
	@echo "✅ 依赖安装完成"

# 检查环境
check-env:
	@echo "🔍 检查编译环境..."
	@if command -v $(CXX) >/dev/null 2>&1; then \
		echo "✅ C++编译器: $(CXX)"; \
		$(CXX) --version | head -1; \
	else \
		echo "❌ 未找到C++编译器"; \
	fi
	@if command -v python3 >/dev/null 2>&1; then \
		echo "✅ Python3: $$(python3 --version)"; \
	else \
		echo "❌ 未找到Python3"; \
	fi
	@if python3 -c "import matplotlib" 2>/dev/null; then \
		echo "✅ matplotlib已安装"; \
	else \
		echo "⚠️ matplotlib未安装"; \
	fi
	@if python3 -c "import pandas" 2>/dev/null; then \
		echo "✅ pandas已安装"; \
	else \
		echo "⚠️ pandas未安装"; \
	fi
	@if python3 -c "import numpy" 2>/dev/null; then \
		echo "✅ numpy已安装"; \
	else \
		echo "⚠️ numpy未安装"; \
	fi

# 显示帮助
help:
	@echo "VCU犁地阻力仿真测试 Makefile"
	@echo ""
	@echo "可用目标:"
	@echo "  all          - 编译程序（默认）"
	@echo "  run          - 编译并运行仿真"
	@echo "  test         - 运行完整测试（包含数据分析）"
	@echo "  clean        - 清理编译文件"
	@echo "  clean-all    - 清理所有文件"
	@echo "  install-deps - 安装Python依赖包"
	@echo "  check-env    - 检查编译环境"
	@echo "  organize     - 整理输出文件"
	@echo "  help         - 显示此帮助信息"
	@echo ""
	@echo "使用示例:"
	@echo "  make test    - 运行完整测试"
	@echo "  make run     - 仅运行仿真"
	@echo "  make clean   - 清理编译文件"

# 声明伪目标
.PHONY: all run test clean clean-all install-deps check-env organize help dirs
