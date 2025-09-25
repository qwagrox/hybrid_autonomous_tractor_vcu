#ifndef LIGHTWEIGHT_ML_PREDICTOR_H
#define LIGHTWEIGHT_ML_PREDICTOR_H

#include "vcu/common/vcu_data_types.h"
#include <cstdint>
#include <array>

namespace vcu {
namespace prediction {

/**
 * @class LightweightMLPredictor
 * @brief 适用于英飞凌MCU的轻量级机器学习负载预测器
 * 
 * 设计目标：
 * - 推理延迟 < 100μs
 * - 内存占用 < 10KB
 * - 预测精度 > 80%
 * - 支持实时更新
 */
class LightweightMLPredictor {
public:
    // 特征数量定义
    static constexpr size_t NUM_FEATURES = 8;
    static constexpr size_t LOOKUP_TABLE_SIZE = 256;
    static constexpr size_t HISTORY_WINDOW_SIZE = 10;
    
    // 预测模型类型
    enum class ModelType {
        LINEAR_REGRESSION,    // 线性回归
        LOOKUP_TABLE,        // 查找表 + 插值
        DECISION_TREE,       // 轻量级决策树
        HYBRID              // 混合模型
    };
    
    /**
     * @brief 构造函数
     */
    LightweightMLPredictor();
    
    /**
     * @brief 初始化预测器
     * @param model_type 模型类型
     * @return 初始化结果
     */
    bool initialize(ModelType model_type = ModelType::HYBRID);
    
    /**
     * @brief 预测负载
     * @param perception_data 感知数据
     * @param current_cvt_ratio 当前CVT传动比
     * @return 预测的负载因子 (0.0-1.0)
     */
    float predict_load(const common::PerceptionData& perception_data, 
                      float current_cvt_ratio);
    
    /**
     * @brief 快速预测 (超低延迟版本)
     * @param speed 车速 (m/s)
     * @param engine_load 发动机负载 (%)
     * @param terrain 地形类型
     * @return 预测负载
     */
    float predict_fast(float speed, float engine_load, common::TerrainType terrain);
    
    /**
     * @brief 更新模型参数 (从域控获取)
     * @param parameters 新的模型参数
     * @return 更新结果
     */
    bool update_parameters(const float* parameters, size_t param_count);
    
    /**
     * @brief 在线学习更新
     * @param features 输入特征
     * @param actual_load 实际负载
     */
    void online_update(const float* features, float actual_load);
    
    /**
     * @brief 获取预测置信度
     * @return 置信度 (0.0-1.0)
     */
    float get_confidence() const;
    
    /**
     * @brief 获取模型性能统计
     */
    struct ModelStats {
        uint32_t prediction_count;
        float average_error;
        float max_error;
        uint32_t inference_time_us;
    };
    
    ModelStats get_statistics() const;

private:
    // 线性回归模型
    struct LinearModel {
        std::array<float, NUM_FEATURES> weights;
        float bias;
        float learning_rate;
        
        LinearModel() : bias(0.0f), learning_rate(0.001f) {
            weights.fill(0.0f);
        }
        
        float predict(const float* features) const {
            float result = bias;
            for(size_t i = 0; i < NUM_FEATURES; ++i) {
                result += weights[i] * features[i];
            }
            return result;
        }
        
        void update(const float* features, float error) {
            bias -= learning_rate * error;
            for(size_t i = 0; i < NUM_FEATURES; ++i) {
                weights[i] -= learning_rate * error * features[i];
            }
        }
    };
    
    // 查找表模型
    struct LookupTableModel {
        struct TableEntry {
            float speed_mps;
            float engine_load;
            uint8_t terrain_code;
            float predicted_load;
        };
        
        std::array<TableEntry, LOOKUP_TABLE_SIZE> table;
        size_t entry_count;
        
        LookupTableModel() : entry_count(0) {}
        
        float predict(float speed, float engine_load, uint8_t terrain) const;
        void build_table(const float* training_data, size_t data_count);
    };
    
    // 轻量级决策树
    struct DecisionTreeModel {
        struct Node {
            uint8_t feature_index;    // 特征索引
            float threshold;          // 分割阈值
            float value;             // 叶节点值
            uint8_t left_child;      // 左子节点索引 (0表示叶节点)
            uint8_t right_child;     // 右子节点索引
        };
        
        static constexpr size_t MAX_NODES = 31;  // 最大31个节点 (深度5的完全二叉树)
        std::array<Node, MAX_NODES> nodes;
        size_t node_count;
        
        DecisionTreeModel() : node_count(0) {}
        
        float predict(const float* features) const;
        void build_tree(const float* training_data, const float* labels, size_t data_count);
    };
    
    // 特征提取
    void extract_features(const common::PerceptionData& perception_data,
                         float current_cvt_ratio,
                         float* features) const;
    
    // 特征归一化
    void normalize_features(float* features) const;
    
    // 历史数据管理
    void update_history(float prediction, float actual_load);
    
    // 置信度计算
    float calculate_confidence(const float* features) const;
    
    // 成员变量
    ModelType current_model_type_;
    
    // 模型实例
    LinearModel linear_model_;
    LookupTableModel lookup_model_;
    DecisionTreeModel tree_model_;
    
    // 特征统计 (用于归一化)
    struct FeatureStats {
        std::array<float, NUM_FEATURES> mean;
        std::array<float, NUM_FEATURES> std_dev;
        std::array<float, NUM_FEATURES> min_val;
        std::array<float, NUM_FEATURES> max_val;
    } feature_stats_;
    
    // 历史数据
    struct HistoryEntry {
        float prediction;
        float actual_load;
        uint32_t timestamp_ms;
    };
    std::array<HistoryEntry, HISTORY_WINDOW_SIZE> history_;
    size_t history_index_;
    
    // 性能统计
    mutable ModelStats stats_;
    
    // 运行时状态
    bool is_initialized_;
    float last_confidence_;
    uint32_t update_count_;
};

/**
 * @class HybridMLPredictor  
 * @brief 混合部署的ML预测器
 * 
 * 结合本地轻量级模型和远程复杂模型
 */
class HybridMLPredictor {
public:
    struct PredictionResult {
        float immediate_prediction;   // 本地快速预测
        float optimized_prediction;   // 远程优化预测
        float confidence_score;       // 置信度
        uint32_t inference_time_us;   // 推理时间
    };
    
    HybridMLPredictor();
    
    bool initialize();
    
    PredictionResult predict(const common::PerceptionData& perception_data,
                           float current_cvt_ratio);
    
    // 从域控获取优化预测结果
    void set_remote_prediction(float prediction, float confidence);
    
    // 检查是否有新的远程预测结果
    bool has_remote_update() const;
    
private:
    LightweightMLPredictor local_predictor_;
    
    // 远程预测结果缓存
    struct RemoteResult {
        float prediction;
        float confidence;
        uint32_t timestamp_ms;
        bool is_valid;
    } remote_result_;
    
    // 预测融合策略
    float fuse_predictions(float local_pred, float remote_pred, 
                          float local_conf, float remote_conf) const;
};

} // namespace prediction
} // namespace vcu

#endif // LIGHTWEIGHT_ML_PREDICTOR_H
