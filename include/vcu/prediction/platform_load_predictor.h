// Copyright 2025 Manus AI

#ifndef VCU_PREDICTION_PLATFORM_LOAD_PREDICTOR_H_
#define VCU_PREDICTION_PLATFORM_LOAD_PREDICTOR_H_

#include "vcu/common/vcu_data_types.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/platform/time_interface.h"
#include "vcu/platform/mutex_interface.h"
#include <vector>
#include <memory>

namespace vcu {
namespace prediction {

/**
 * @brief 负载预测结果枚举
 */
enum class PredictionResult {
    SUCCESS = 0,
    ERROR_NOT_INITIALIZED,
    ERROR_INVALID_INPUT,
    ERROR_INSUFFICIENT_DATA,
    ERROR_COMPUTATION_FAILED
};

/**
 * @brief 负载预测配置
 */
struct LoadPredictionConfig {
    uint32_t history_window_size = 100;    // 历史数据窗口大小
    uint32_t prediction_horizon_ms = 5000; // 预测时间范围(毫秒)
    float terrain_weight = 0.4f;           // 地形因子权重
    float speed_weight = 0.3f;             // 速度因子权重
    float load_weight = 0.3f;              // 当前负载因子权重
};

/**
 * @brief 历史数据点
 */
struct LoadDataPoint {
    uint64_t timestamp_ms;                  // 时间戳(毫秒)
    float vehicle_speed_kmh;                // 车辆速度
    float engine_load_percent;              // 发动机负载
    common::TerrainType terrain_type;       // 地形类型
    float load_factor;                      // 负载因子
};

/**
 * @brief 负载预测结果
 */
struct LoadPrediction {
    float predicted_load_percent;           // 预测负载百分比
    float confidence_level;                 // 置信度 (0.0-1.0)
    uint32_t prediction_horizon_ms;         // 预测时间范围
    common::TerrainType expected_terrain;   // 预期地形类型
};

/**
 * @brief 使用平台抽象层的负载预测器
 * 
 * 基于历史数据和当前状态预测未来的发动机负载需求
 */
class PlatformLoadPredictor {
public:
    explicit PlatformLoadPredictor(PlatformInterface* platform);
    explicit PlatformLoadPredictor(PlatformInterface* platform, const LoadPredictionConfig& config);
    ~PlatformLoadPredictor();

    /**
     * @brief 初始化预测器
     */
    PredictionResult initialize();

    /**
     * @brief 关闭预测器
     */
    PredictionResult shutdown();

    /**
     * @brief 更新感知数据
     */
    PredictionResult update_perception_data(const common::PerceptionData& data);

    /**
     * @brief 预测负载
     */
    PredictionResult predict_load(LoadPrediction& prediction);

    /**
     * @brief 获取历史数据统计
     */
    PredictionResult get_statistics(float& avg_load, float& max_load, uint32_t& data_points);

    /**
     * @brief 清除历史数据
     */
    void clear_history();

    /**
     * @brief 检查是否已初始化
     */
    bool is_initialized() const;

private:
    float calculate_terrain_factor(common::TerrainType terrain) const;
    float calculate_speed_factor(float speed_kmh) const;
    float calculate_load_trend() const;
    float calculate_confidence(uint32_t data_points, float variance) const;
    common::TerrainType predict_terrain_type() const;

    PlatformInterface* platform_;
    std::unique_ptr<TimeInterface> time_interface_;
    std::unique_ptr<MutexInterface> data_mutex_;
    
    LoadPredictionConfig config_;
    std::vector<LoadDataPoint> history_;
    uint64_t last_update_time_ms_;
    bool is_initialized_;
};

} // namespace prediction
} // namespace vcu

#endif // VCU_PREDICTION_PLATFORM_LOAD_PREDICTOR_H_
