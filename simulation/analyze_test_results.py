#!/usr/bin/env python3
"""
VCU犁地阻力测试结果分析脚本
分析测试数据并生成可视化报告
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import json

class PlowResistanceTestAnalyzer:
    def __init__(self, csv_file="vcu_decision_log.csv"):
        """初始化分析器"""
        self.csv_file = csv_file
        self.data = None
        self.analysis_results = {}
        
    def load_data(self):
        """加载测试数据"""
        try:
            self.data = pd.read_csv(self.csv_file)
            print(f"✅ 成功加载测试数据: {len(self.data)} 条记录")
            return True
        except FileNotFoundError:
            print(f"❌ 未找到测试数据文件: {self.csv_file}")
            return False
        except Exception as e:
            print(f"❌ 数据加载失败: {e}")
            return False
    
    def analyze_anomaly_detection(self):
        """分析异常检测性能"""
        print("\n=== 异常检测分析 ===")
        
        # 查找状态变化点
        state_changes = self.data[self.data['State'].diff() != 0]
        
        # 查找从状态0到状态1的转换（开始检测异常）
        detection_start = state_changes[
            (state_changes['State'] == 1) & 
            (state_changes['State'].shift(1) == 0)
        ]
        
        if not detection_start.empty:
            detection_time = detection_start.iloc[0]['Time'] - 30000  # 30秒是异常开始时间
            detection_time_sec = detection_time / 1000.0
            
            self.analysis_results['anomaly_detection_time'] = detection_time_sec
            
            print(f"异常检测时间: {detection_time_sec:.2f}s")
            
            if detection_time_sec <= 2.0:
                print("✅ 检测时间符合要求 (≤2.0s)")
            else:
                print("❌ 检测时间超出要求 (>2.0s)")
        else:
            print("❌ 未检测到异常检测事件")
            self.analysis_results['anomaly_detection_time'] = -1
    
    def analyze_torque_response(self):
        """分析发动机扭矩响应"""
        print("\n=== 发动机扭矩响应分析 ===")
        
        # 计算基线扭矩（前30秒的平均值）
        baseline_data = self.data[self.data['Time'] < 30000]
        baseline_torque = baseline_data['EngTorque'].mean()
        
        # 计算最大扭矩
        max_torque = self.data['EngTorque'].max()
        torque_increase = (max_torque - baseline_torque) / baseline_torque
        
        self.analysis_results['baseline_torque'] = baseline_torque
        self.analysis_results['max_torque'] = max_torque
        self.analysis_results['torque_increase'] = torque_increase
        
        print(f"基线扭矩: {baseline_torque:.1f}Nm")
        print(f"最大扭矩: {max_torque:.1f}Nm")
        print(f"扭矩增加: {torque_increase*100:.1f}%")
        
        if torque_increase >= 0.20:
            print("✅ 扭矩响应符合要求 (≥20%)")
        else:
            print("❌ 扭矩响应不足 (<20%)")
    
    def analyze_depth_adjustment(self):
        """分析犁地深度调整"""
        print("\n=== 犁地深度调整分析 ===")
        
        # 计算基线深度
        baseline_data = self.data[self.data['Time'] < 30000]
        baseline_depth = baseline_data['PlowDepth'].mean()
        
        # 计算最小深度
        min_depth = self.data['PlowDepth'].min()
        depth_reduction = (baseline_depth - min_depth) / baseline_depth
        
        self.analysis_results['baseline_depth'] = baseline_depth
        self.analysis_results['min_depth'] = min_depth
        self.analysis_results['depth_reduction'] = depth_reduction
        
        print(f"基线深度: {baseline_depth:.3f}m")
        print(f"最小深度: {min_depth:.3f}m")
        print(f"深度减少: {depth_reduction*100:.1f}%")
        
        if depth_reduction >= 0.15:
            print("✅ 深度调整符合要求 (≥15%)")
        else:
            print("❌ 深度调整不足 (<15%)")
    
    def analyze_hybrid_activation(self):
        """分析混合动力激活"""
        print("\n=== 混合动力激活分析 ===")
        
        # 检查是否有电机扭矩输出
        motor_active = self.data['MotorTorque'].max() > 0
        
        if motor_active:
            max_motor_torque = self.data['MotorTorque'].max()
            activation_time = self.data[self.data['MotorTorque'] > 0].iloc[0]['Time']
            
            self.analysis_results['hybrid_activated'] = True
            self.analysis_results['max_motor_torque'] = max_motor_torque
            self.analysis_results['hybrid_activation_time'] = activation_time / 1000.0
            
            print(f"✅ 混合动力模式已激活")
            print(f"最大电机扭矩: {max_motor_torque:.1f}Nm")
            print(f"激活时间: {activation_time/1000.0:.1f}s")
        else:
            print("❌ 混合动力模式未激活")
            self.analysis_results['hybrid_activated'] = False
    
    def analyze_recovery_performance(self):
        """分析系统恢复性能"""
        print("\n=== 系统恢复性能分析 ===")
        
        # 查找恢复开始时间（状态变为4）
        recovery_start = self.data[self.data['State'] == 4]
        
        if not recovery_start.empty:
            recovery_start_time = recovery_start.iloc[0]['Time'] / 1000.0
            
            # 查找恢复完成时间（状态变为5）
            recovery_complete = self.data[self.data['State'] == 5]
            
            if not recovery_complete.empty:
                recovery_complete_time = recovery_complete.iloc[0]['Time'] / 1000.0
                recovery_duration = recovery_complete_time - recovery_start_time
                
                self.analysis_results['recovery_start_time'] = recovery_start_time
                self.analysis_results['recovery_complete_time'] = recovery_complete_time
                self.analysis_results['recovery_duration'] = recovery_duration
                
                print(f"恢复开始时间: {recovery_start_time:.1f}s")
                print(f"恢复完成时间: {recovery_complete_time:.1f}s")
                print(f"恢复持续时间: {recovery_duration:.1f}s")
                
                if recovery_duration <= 30.0:
                    print("✅ 恢复时间符合要求 (≤30s)")
                else:
                    print("❌ 恢复时间过长 (>30s)")
            else:
                print("⚠️ 未检测到恢复完成")
        else:
            print("❌ 未检测到恢复开始")
    
    def generate_visualizations(self):
        """生成可视化图表"""
        print("\n=== 生成可视化图表 ===")
        
        # 设置中文字体
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False
        
        # 创建子图
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('VCU犁地阻力测试结果分析', fontsize=16, fontweight='bold')
        
        # 时间轴（转换为秒）
        time_sec = self.data['Time'] / 1000.0
        
        # 图1: 发动机扭矩时间序列
        axes[0, 0].plot(time_sec, self.data['EngTorque'], 'b-', linewidth=2)
        axes[0, 0].axhline(y=self.analysis_results.get('baseline_torque', 240), 
                          color='g', linestyle='--', label='基线扭矩')
        axes[0, 0].axhline(y=self.analysis_results.get('max_torque', 312), 
                          color='r', linestyle='--', label='最大扭矩')
        axes[0, 0].set_title('发动机扭矩响应')
        axes[0, 0].set_xlabel('时间 (s)')
        axes[0, 0].set_ylabel('扭矩 (Nm)')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # 图2: 犁地深度调整
        axes[0, 1].plot(time_sec, self.data['PlowDepth'], 'g-', linewidth=2)
        axes[0, 1].axhline(y=self.analysis_results.get('baseline_depth', 0.25), 
                          color='b', linestyle='--', label='基线深度')
        axes[0, 1].axhline(y=self.analysis_results.get('min_depth', 0.20), 
                          color='r', linestyle='--', label='最小深度')
        axes[0, 1].set_title('犁地深度调整')
        axes[0, 1].set_xlabel('时间 (s)')
        axes[0, 1].set_ylabel('深度 (m)')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # 图3: VCU决策状态
        axes[1, 0].plot(time_sec, self.data['State'], 'r-', linewidth=2, marker='o', markersize=3)
        axes[1, 0].set_title('VCU决策状态变化')
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('状态编号')
        axes[1, 0].set_yticks([0, 1, 2, 3, 4, 5])
        axes[1, 0].set_yticklabels(['正常', '检测', '响应', '调整', '恢复', '优化'])
        axes[1, 0].grid(True, alpha=0.3)
        
        # 图4: 电机扭矩输出
        axes[1, 1].plot(time_sec, self.data['MotorTorque'], 'm-', linewidth=2)
        axes[1, 1].fill_between(time_sec, 0, self.data['MotorTorque'], alpha=0.3)
        axes[1, 1].set_title('电机辅助扭矩')
        axes[1, 1].set_xlabel('时间 (s)')
        axes[1, 1].set_ylabel('扭矩 (Nm)')
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('plow_resistance_test_analysis.png', dpi=300, bbox_inches='tight')
        print("✅ 可视化图表已保存: plow_resistance_test_analysis.png")
        
        # 显示图表（如果在交互环境中）
        try:
            plt.show()
        except:
            pass
    
    def calculate_overall_score(self):
        """计算总体评分"""
        print("\n=== 总体评分计算 ===")
        
        score = 0
        max_score = 100
        
        # 异常检测时间 (20分)
        detection_time = self.analysis_results.get('anomaly_detection_time', -1)
        if detection_time > 0:
            if detection_time <= 1.5:
                score += 20
            elif detection_time <= 2.0:
                score += 15
            else:
                score += 5
        
        # 发动机扭矩响应 (25分)
        torque_increase = self.analysis_results.get('torque_increase', 0)
        if torque_increase >= 0.25:
            score += 25
        elif torque_increase >= 0.20:
            score += 20
        elif torque_increase >= 0.15:
            score += 10
        
        # 犁地深度调整 (20分)
        depth_reduction = self.analysis_results.get('depth_reduction', 0)
        if depth_reduction >= 0.20:
            score += 20
        elif depth_reduction >= 0.15:
            score += 15
        elif depth_reduction >= 0.10:
            score += 10
        
        # 混合动力激活 (15分)
        if self.analysis_results.get('hybrid_activated', False):
            score += 15
        
        # 系统恢复时间 (15分)
        recovery_duration = self.analysis_results.get('recovery_duration', 999)
        if recovery_duration <= 25:
            score += 15
        elif recovery_duration <= 30:
            score += 12
        elif recovery_duration <= 40:
            score += 8
        
        # 系统稳定性 (5分) - 简化评估
        score += 5  # 假设系统稳定
        
        self.analysis_results['overall_score'] = score
        
        print(f"总体评分: {score}/{max_score}")
        
        if score >= 90:
            grade = "优秀 ⭐⭐⭐⭐⭐"
        elif score >= 80:
            grade = "良好 ⭐⭐⭐⭐"
        elif score >= 70:
            grade = "合格 ⭐⭐⭐"
        elif score >= 60:
            grade = "及格 ⭐⭐"
        else:
            grade = "不及格 ⭐"
        
        print(f"评级: {grade}")
        
        return score >= 70  # 70分及格
    
    def generate_json_report(self):
        """生成JSON格式的分析报告"""
        report = {
            "test_info": {
                "test_name": "犁地阻力智能决策测试",
                "test_date": datetime.now().isoformat(),
                "data_file": self.csv_file,
                "total_records": len(self.data) if self.data is not None else 0
            },
            "analysis_results": self.analysis_results,
            "recommendations": self.generate_recommendations()
        }
        
        with open('plow_resistance_test_report.json', 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        print("✅ JSON报告已生成: plow_resistance_test_report.json")
    
    def generate_recommendations(self):
        """生成改进建议"""
        recommendations = []
        
        # 基于分析结果生成建议
        detection_time = self.analysis_results.get('anomaly_detection_time', -1)
        if detection_time > 2.0:
            recommendations.append("建议优化异常检测算法，提高检测速度")
        
        torque_increase = self.analysis_results.get('torque_increase', 0)
        if torque_increase < 0.20:
            recommendations.append("建议增加发动机扭矩储备或优化扭矩控制策略")
        
        depth_reduction = self.analysis_results.get('depth_reduction', 0)
        if depth_reduction < 0.15:
            recommendations.append("建议改进犁地深度自适应控制算法")
        
        if not self.analysis_results.get('hybrid_activated', False):
            recommendations.append("建议检查混合动力系统激活逻辑")
        
        recovery_duration = self.analysis_results.get('recovery_duration', 999)
        if recovery_duration > 30:
            recommendations.append("建议优化系统恢复策略，缩短恢复时间")
        
        if not recommendations:
            recommendations.append("系统表现优秀，建议继续保持当前性能水平")
        
        return recommendations
    
    def run_complete_analysis(self):
        """运行完整分析流程"""
        print("🔍 开始VCU犁地阻力测试结果分析")
        print("=" * 50)
        
        # 加载数据
        if not self.load_data():
            return False
        
        # 执行各项分析
        self.analyze_anomaly_detection()
        self.analyze_torque_response()
        self.analyze_depth_adjustment()
        self.analyze_hybrid_activation()
        self.analyze_recovery_performance()
        
        # 生成可视化
        self.generate_visualizations()
        
        # 计算总体评分
        passed = self.calculate_overall_score()
        
        # 生成报告
        self.generate_json_report()
        
        print("\n" + "=" * 50)
        print(f"📊 分析完成，测试结果: {'✅ 通过' if passed else '❌ 失败'}")
        
        return passed

def main():
    """主函数"""
    analyzer = PlowResistanceTestAnalyzer()
    analyzer.run_complete_analysis()

if __name__ == "__main__":
    main()
