#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VCU动力总成仿真数据可视化分析工具
用于分析和可视化动力总成仿真数据
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from datetime import datetime

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class PowertrainDataAnalyzer:
    def __init__(self, csv_file):
        """初始化分析器"""
        self.csv_file = csv_file
        self.data = None
        self.load_data()
    
    def load_data(self):
        """加载CSV数据"""
        try:
            self.data = pd.read_csv(self.csv_file)
            print(f"✅ 成功加载数据: {len(self.data)} 行")
            print(f"📊 时间范围: {self.data['Timestamp'].min():.1f}s - {self.data['Timestamp'].max():.1f}s")
        except Exception as e:
            print(f"❌ 数据加载失败: {e}")
            sys.exit(1)
    
    def analyze_basic_stats(self):
        """基础统计分析"""
        print("\n=== 基础统计分析 ===")
        
        # 发动机统计
        print(f"发动机转速: {self.data['EngineSpeed'].mean():.1f} ± {self.data['EngineSpeed'].std():.1f} RPM")
        print(f"发动机负载: {self.data['EngineLoad'].mean():.1f} ± {self.data['EngineLoad'].std():.1f} %")
        print(f"发动机功率: {self.data['EnginePower'].mean():.1f} ± {self.data['EnginePower'].std():.1f} kW")
        print(f"燃油消耗: {self.data['FuelFlowRate'].mean():.1f} ± {self.data['FuelFlowRate'].std():.1f} L/h")
        
        # 电机统计
        motor_active = self.data[self.data['MotorPower'].abs() > 1.0]
        if len(motor_active) > 0:
            print(f"电机功率 (激活时): {motor_active['MotorPower'].mean():.1f} ± {motor_active['MotorPower'].std():.1f} kW")
            print(f"电机效率 (激活时): {motor_active['MotorEfficiency'].mean():.1f} ± {motor_active['MotorEfficiency'].std():.1f} %")
        
        # 电池统计
        print(f"电池SOC: {self.data['BatterySOC'].mean():.1f} ± {self.data['BatterySOC'].std():.1f} %")
        print(f"电池功率: {self.data['BatteryPower'].mean():.1f} ± {self.data['BatteryPower'].std():.1f} kW")
        
        # CVT统计
        print(f"CVT传动比: {self.data['CVTRatio'].mean():.2f} ± {self.data['CVTRatio'].std():.2f}")
        print(f"CVT效率: {self.data['CVTEfficiency'].mean():.1f} ± {self.data['CVTEfficiency'].std():.1f} %")
        
        # 系统统计
        print(f"系统总功率: {self.data['TotalOutputPower'].mean():.1f} ± {self.data['TotalOutputPower'].std():.1f} kW")
        print(f"系统效率: {self.data['SystemEfficiency'].mean():.1f} ± {self.data['SystemEfficiency'].std():.1f} %")
    
    def detect_operating_modes(self):
        """检测运行模式"""
        print("\n=== 运行模式分析 ===")
        
        mode_counts = self.data['PowertrainMode'].value_counts()
        mode_names = {0: '发动机模式', 1: '发电模式', 2: '混合模式'}
        
        for mode, count in mode_counts.items():
            percentage = count / len(self.data) * 100
            mode_name = mode_names.get(mode, f'模式{mode}')
            print(f"{mode_name}: {count} 次 ({percentage:.1f}%)")
        
        # 分析模式切换
        mode_changes = (self.data['PowertrainMode'].diff() != 0).sum()
        print(f"模式切换次数: {mode_changes}")
    
    def analyze_efficiency(self):
        """效率分析"""
        print("\n=== 效率分析 ===")
        
        # 发动机效率分析
        engine_power = self.data['EnginePower']
        fuel_flow = self.data['FuelFlowRate']
        
        # 计算发动机热效率 (简化计算)
        fuel_energy_rate = fuel_flow * 35.3  # kW (柴油热值约35.3 MJ/L)
        engine_efficiency = np.where(fuel_energy_rate > 0, 
                                   engine_power / fuel_energy_rate * 100, 0)
        
        print(f"发动机热效率: {engine_efficiency.mean():.1f} ± {engine_efficiency.std():.1f} %")
        
        # 混合动力系统效率
        hybrid_data = self.data[self.data['PowertrainMode'] == 2]
        if len(hybrid_data) > 0:
            print(f"混合模式系统效率: {hybrid_data['SystemEfficiency'].mean():.1f} %")
        
        # 能量回收分析
        regen_data = self.data[self.data['MotorPower'] < -1.0]  # 负功率表示发电
        if len(regen_data) > 0:
            total_regen_energy = (-regen_data['MotorPower'] * 0.1 / 3600).sum()  # kWh
            print(f"能量回收: {total_regen_energy:.2f} kWh")
    
    def create_comprehensive_plot(self):
        """创建综合分析图表"""
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle('VCU动力总成仿真数据分析', fontsize=16, fontweight='bold')
        
        time = self.data['Timestamp']
        
        # 1. 功率分析
        ax1 = axes[0, 0]
        ax1.plot(time, self.data['EnginePower'], 'b-', label='发动机功率', linewidth=1.5)
        ax1.plot(time, self.data['MotorPower'], 'r-', label='电机功率', linewidth=1.5)
        ax1.plot(time, self.data['TotalOutputPower'], 'g-', label='总输出功率', linewidth=2)
        ax1.set_xlabel('时间 (s)')
        ax1.set_ylabel('功率 (kW)')
        ax1.set_title('功率分析')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. 发动机运行状态
        ax2 = axes[0, 1]
        ax2.plot(time, self.data['EngineSpeed'], 'b-', label='转速', linewidth=1.5)
        ax2_twin = ax2.twinx()
        ax2_twin.plot(time, self.data['EngineLoad'], 'r-', label='负载', linewidth=1.5)
        ax2.set_xlabel('时间 (s)')
        ax2.set_ylabel('转速 (RPM)', color='b')
        ax2_twin.set_ylabel('负载 (%)', color='r')
        ax2.set_title('发动机运行状态')
        ax2.grid(True, alpha=0.3)
        
        # 3. 电池状态
        ax3 = axes[1, 0]
        ax3.plot(time, self.data['BatterySOC'], 'g-', label='SOC', linewidth=2)
        ax3_twin = ax3.twinx()
        ax3_twin.plot(time, self.data['BatteryPower'], 'orange', label='功率', linewidth=1.5)
        ax3.set_xlabel('时间 (s)')
        ax3.set_ylabel('SOC (%)', color='g')
        ax3_twin.set_ylabel('功率 (kW)', color='orange')
        ax3.set_title('电池状态')
        ax3.grid(True, alpha=0.3)
        
        # 4. CVT传动系统
        ax4 = axes[1, 1]
        ax4.plot(time, self.data['CVTRatio'], 'purple', label='传动比', linewidth=1.5)
        ax4_twin = ax4.twinx()
        ax4_twin.plot(time, self.data['CVTEfficiency'], 'brown', label='效率', linewidth=1.5)
        ax4.set_xlabel('时间 (s)')
        ax4.set_ylabel('传动比', color='purple')
        ax4_twin.set_ylabel('效率 (%)', color='brown')
        ax4.set_title('CVT传动系统')
        ax4.grid(True, alpha=0.3)
        
        # 5. 温度监控
        ax5 = axes[2, 0]
        ax5.plot(time, self.data['CoolantTemp'], 'b-', label='冷却液温度', linewidth=1.5)
        ax5.plot(time, self.data['MotorTemp'], 'r-', label='电机温度', linewidth=1.5)
        ax5.plot(time, self.data['BatteryTemp'], 'g-', label='电池温度', linewidth=1.5)
        ax5.plot(time, self.data['CVTOilTemp'], 'orange', label='CVT油温', linewidth=1.5)
        ax5.set_xlabel('时间 (s)')
        ax5.set_ylabel('温度 (°C)')
        ax5.set_title('温度监控')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 6. 运行模式和效率
        ax6 = axes[2, 1]
        mode_colors = {0: 'blue', 1: 'orange', 2: 'red'}
        for mode in self.data['PowertrainMode'].unique():
            mask = self.data['PowertrainMode'] == mode
            mode_name = {0: '发动机', 1: '发电', 2: '混合'}[mode]
            ax6.scatter(time[mask], self.data.loc[mask, 'SystemEfficiency'], 
                       c=mode_colors[mode], label=f'{mode_name}模式', alpha=0.6, s=10)
        ax6.set_xlabel('时间 (s)')
        ax6.set_ylabel('系统效率 (%)')
        ax6.set_title('运行模式与效率')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig
    
    def create_efficiency_analysis_plot(self):
        """创建效率分析图表"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('动力总成效率分析', fontsize=16, fontweight='bold')
        
        # 1. 发动机效率图
        ax1 = axes[0, 0]
        engine_power = self.data['EnginePower']
        fuel_flow = self.data['FuelFlowRate']
        fuel_energy_rate = fuel_flow * 35.3 / 3600 * 1000  # kW
        engine_efficiency = np.where(fuel_energy_rate > 0, 
                                   engine_power / fuel_energy_rate * 100, 0)
        
        scatter = ax1.scatter(self.data['EngineSpeed'], self.data['EngineLoad'], 
                            c=engine_efficiency, cmap='viridis', alpha=0.6, s=20)
        ax1.set_xlabel('发动机转速 (RPM)')
        ax1.set_ylabel('发动机负载 (%)')
        ax1.set_title('发动机效率图')
        plt.colorbar(scatter, ax=ax1, label='效率 (%)')
        
        # 2. 电机效率图
        ax2 = axes[0, 1]
        motor_active = self.data[self.data['MotorPower'].abs() > 1.0]
        if len(motor_active) > 0:
            scatter2 = ax2.scatter(motor_active['MotorSpeed'], motor_active['MotorTorque'].abs(), 
                                 c=motor_active['MotorEfficiency'], cmap='plasma', alpha=0.6, s=20)
            ax2.set_xlabel('电机转速 (RPM)')
            ax2.set_ylabel('电机扭矩 (Nm)')
            ax2.set_title('电机效率图')
            plt.colorbar(scatter2, ax=ax2, label='效率 (%)')
        else:
            ax2.text(0.5, 0.5, '无电机运行数据', ha='center', va='center', transform=ax2.transAxes)
            ax2.set_title('电机效率图')
        
        # 3. 系统效率时间序列
        ax3 = axes[1, 0]
        time = self.data['Timestamp']
        ax3.plot(time, self.data['SystemEfficiency'], 'g-', linewidth=1.5)
        ax3.set_xlabel('时间 (s)')
        ax3.set_ylabel('系统效率 (%)')
        ax3.set_title('系统效率时间序列')
        ax3.grid(True, alpha=0.3)
        
        # 4. 功率分配分析
        ax4 = axes[1, 1]
        hybrid_data = self.data[self.data['PowertrainMode'] == 2]
        if len(hybrid_data) > 0:
            ax4.plot(hybrid_data['Timestamp'], hybrid_data['PowerSplitRatio'], 
                    'purple', linewidth=1.5, label='发动机功率比例')
            ax4.plot(hybrid_data['Timestamp'], 1 - hybrid_data['PowerSplitRatio'], 
                    'orange', linewidth=1.5, label='电机功率比例')
            ax4.set_xlabel('时间 (s)')
            ax4.set_ylabel('功率比例')
            ax4.set_title('混合模式功率分配')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
        else:
            ax4.text(0.5, 0.5, '无混合模式数据', ha='center', va='center', transform=ax4.transAxes)
            ax4.set_title('混合模式功率分配')
        
        plt.tight_layout()
        return fig
    
    def generate_report(self):
        """生成分析报告"""
        report_filename = self.csv_file.replace('.csv', '_analysis_report.txt')
        
        with open(report_filename, 'w', encoding='utf-8') as f:
            f.write("VCU动力总成仿真数据分析报告\n")
            f.write("=" * 50 + "\n")
            f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"数据文件: {self.csv_file}\n")
            f.write(f"数据点数: {len(self.data)}\n")
            f.write(f"时间范围: {self.data['Timestamp'].min():.1f}s - {self.data['Timestamp'].max():.1f}s\n\n")
            
            # 基础统计
            f.write("基础统计信息:\n")
            f.write("-" * 30 + "\n")
            f.write(f"发动机平均转速: {self.data['EngineSpeed'].mean():.1f} RPM\n")
            f.write(f"发动机平均负载: {self.data['EngineLoad'].mean():.1f} %\n")
            f.write(f"发动机平均功率: {self.data['EnginePower'].mean():.1f} kW\n")
            f.write(f"平均燃油消耗: {self.data['FuelFlowRate'].mean():.1f} L/h\n")
            f.write(f"电池平均SOC: {self.data['BatterySOC'].mean():.1f} %\n")
            f.write(f"系统平均效率: {self.data['SystemEfficiency'].mean():.1f} %\n\n")
            
            # 运行模式分析
            f.write("运行模式分析:\n")
            f.write("-" * 30 + "\n")
            mode_counts = self.data['PowertrainMode'].value_counts()
            mode_names = {0: '发动机模式', 1: '发电模式', 2: '混合模式'}
            
            for mode, count in mode_counts.items():
                percentage = count / len(self.data) * 100
                mode_name = mode_names.get(mode, f'模式{mode}')
                f.write(f"{mode_name}: {count} 次 ({percentage:.1f}%)\n")
            
            # 效率分析
            f.write("\n效率分析:\n")
            f.write("-" * 30 + "\n")
            
            # 发动机效率
            engine_power = self.data['EnginePower']
            fuel_flow = self.data['FuelFlowRate']
            fuel_energy_rate = fuel_flow * 35.3 / 3600 * 1000  # kW
            engine_efficiency = np.where(fuel_energy_rate > 0, 
                                       engine_power / fuel_energy_rate * 100, 0)
            f.write(f"发动机平均热效率: {engine_efficiency.mean():.1f} %\n")
            
            # 电机效率
            motor_active = self.data[self.data['MotorPower'].abs() > 1.0]
            if len(motor_active) > 0:
                f.write(f"电机平均效率: {motor_active['MotorEfficiency'].mean():.1f} %\n")
            
            # 能量分析
            f.write("\n能量分析:\n")
            f.write("-" * 30 + "\n")
            
            # 总能量消耗
            dt = self.data['Timestamp'].diff().mean()
            total_fuel_energy = (self.data['FuelFlowRate'] * dt / 3600 * 35.3).sum()  # MJ
            total_electric_energy = (self.data['BatteryPower'].abs() * dt / 3600).sum()  # kWh
            
            f.write(f"总燃油能量: {total_fuel_energy:.1f} MJ\n")
            f.write(f"总电能消耗: {total_electric_energy:.2f} kWh\n")
            
            # SOC变化
            soc_change = self.data['BatterySOC'].iloc[-1] - self.data['BatterySOC'].iloc[0]
            f.write(f"电池SOC变化: {soc_change:.1f} %\n")
        
        print(f"📄 分析报告已生成: {report_filename}")
        return report_filename
    
    def run_complete_analysis(self):
        """运行完整分析"""
        print("🔍 开始动力总成数据分析...")
        
        # 基础统计分析
        self.analyze_basic_stats()
        
        # 运行模式检测
        self.detect_operating_modes()
        
        # 效率分析
        self.analyze_efficiency()
        
        # 生成图表
        print("\n📊 生成分析图表...")
        
        # 综合分析图
        fig1 = self.create_comprehensive_plot()
        plot1_filename = self.csv_file.replace('.csv', '_comprehensive_analysis.png')
        fig1.savefig(plot1_filename, dpi=300, bbox_inches='tight')
        print(f"✅ 综合分析图已保存: {plot1_filename}")
        
        # 效率分析图
        fig2 = self.create_efficiency_analysis_plot()
        plot2_filename = self.csv_file.replace('.csv', '_efficiency_analysis.png')
        fig2.savefig(plot2_filename, dpi=300, bbox_inches='tight')
        print(f"✅ 效率分析图已保存: {plot2_filename}")
        
        # 生成报告
        report_filename = self.generate_report()
        
        print(f"\n🎉 分析完成！")
        print(f"📁 输出文件:")
        print(f"  - 综合分析图: {plot1_filename}")
        print(f"  - 效率分析图: {plot2_filename}")
        print(f"  - 分析报告: {report_filename}")

def main():
    """主函数"""
    print("VCU动力总成仿真数据可视化分析工具")
    print("版本: 1.0")
    print("=" * 50)
    
    # 检查命令行参数
    if len(sys.argv) < 2:
        print("使用方法: python3 visualize_powertrain_data.py <CSV文件>")
        print("示例: python3 visualize_powertrain_data.py powertrain_data_normal_operation.csv")
        
        # 查找可用的CSV文件
        csv_files = [f for f in os.listdir('.') if f.startswith('powertrain_data_') and f.endswith('.csv')]
        if csv_files:
            print(f"\n可用的数据文件:")
            for i, file in enumerate(csv_files, 1):
                print(f"  {i}. {file}")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    
    # 检查文件是否存在
    if not os.path.exists(csv_file):
        print(f"❌ 错误: 文件 '{csv_file}' 不存在")
        sys.exit(1)
    
    # 创建分析器并运行分析
    analyzer = PowertrainDataAnalyzer(csv_file)
    analyzer.run_complete_analysis()

if __name__ == "__main__":
    main()
