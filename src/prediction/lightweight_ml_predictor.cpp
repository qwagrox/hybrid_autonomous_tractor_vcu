#include "vcu/prediction/lightweight_ml_predictor.h"
#include <cmath>
#include <algorithm>
#include <cstring>

namespace vcu {
namespace prediction {

LightweightMLPredictor::LightweightMLPredictor()
    : current_model_type_(ModelType::HYBRID)
    , history_index_(0)
    , is_initialized_(false)
    , last_confidence_(0.0f)
    , update_count_(0) {
    
    // 初始化统计信息
    stats_.prediction_count = 0;
    stats_.average_error = 0.0f;
    stats_.max_error = 0.0f;
    stats_.inference_time_us = 0;
    
    // 初始化特征统计 (基于经验值)
    feature_stats_.mean = {15.0f, 50.0f, 1500.0f, 50.0f, 2.0f, 0.5f, 25.0f, 1.5f};
    feature_stats_.std_dev = {10.0f, 30.0f, 500.0f, 25.0f, 0.5f, 0.3f, 15.0f, 0.5f};
    feature_stats_.min_val = {0.0f, 0.0f, 800.0f, 0.0f, 0.5f, 0.0f, -10.0f, 0.5f};
    feature_stats_.max_val = {30.0f, 100.0f, 3000.0f, 100.0f, 3.0f, 1.0f, 60.0f, 3.0f};
    
    // 初始化历史数据
    for(auto& entry : history_) {
        entry.prediction = 0.0f;
        entry.actual_load = 0.0f;
        entry.timestamp_ms = 0;
    }
}

bool LightweightMLPredictor::initialize(ModelType model_type) {
    current_model_type_ = model_type;
    
    switch(model_type) {
        case ModelType::LINEAR_REGRESSION:
            // 初始化线性模型权重 (基于经验或预训练)
            linear_model_.weights = {
                0.02f,   // 车速影响
                0.008f,  // 发动机负载影响
                -0.0001f, // 发动机转速影响 (负相关)
                0.005f,  // 燃油消耗影响
                0.15f,   // CVT传动比影响
                0.3f,    // 地形因子影响
                0.001f,  // 温度影响
                0.1f     // 历史负载影响
            };
            linear_model_.bias = 0.2f;  // 基础负载
            break;
            
        case ModelType::LOOKUP_TABLE:
            build_default_lookup_table();
            break;
            
        case ModelType::DECISION_TREE:
            build_default_decision_tree();
            break;
            
        case ModelType::HYBRID:
            // 初始化所有模型
            initialize(ModelType::LINEAR_REGRESSION);
            initialize(ModelType::LOOKUP_TABLE);
            initialize(ModelType::DECISION_TREE);
            break;
    }
    
    is_initialized_ = true;
    return true;
}

float LightweightMLPredictor::predict_load(const common::PerceptionData& perception_data,
                                          float current_cvt_ratio) {
    if(!is_initialized_) {
        return 0.5f;  // 默认中等负载
    }
    
    // 记录开始时间 (简化的时间测量)
    uint32_t start_time = get_timestamp_us();
    
    // 提取特征
    float features[NUM_FEATURES];
    extract_features(perception_data, current_cvt_ratio, features);
    
    // 归一化特征
    normalize_features(features);
    
    float prediction = 0.0f;
    
    switch(current_model_type_) {
        case ModelType::LINEAR_REGRESSION:
            prediction = linear_model_.predict(features);
            break;
            
        case ModelType::LOOKUP_TABLE:
            prediction = lookup_model_.predict(
                perception_data.vehicle_speed_kmh / 3.6f,  // 转换为m/s
                perception_data.engine_load_percent,
                static_cast<uint8_t>(perception_data.terrain_type)
            );
            break;
            
        case ModelType::DECISION_TREE:
            prediction = tree_model_.predict(features);
            break;
            
        case ModelType::HYBRID:
            // 多模型融合
            float linear_pred = linear_model_.predict(features);
            float lookup_pred = lookup_model_.predict(
                perception_data.vehicle_speed_kmh / 3.6f,
                perception_data.engine_load_percent,
                static_cast<uint8_t>(perception_data.terrain_type)
            );
            float tree_pred = tree_model_.predict(features);
            
            // 加权平均 (可以根据历史性能动态调整权重)
            prediction = 0.5f * linear_pred + 0.3f * lookup_pred + 0.2f * tree_pred;
            break;
    }
    
    // 限制预测结果范围
    prediction = std::max(0.0f, std::min(1.0f, prediction));
    
    // 计算置信度
    last_confidence_ = calculate_confidence(features);
    
    // 更新统计信息
    uint32_t inference_time = get_timestamp_us() - start_time;
    stats_.inference_time_us = inference_time;
    stats_.prediction_count++;
    
    return prediction;
}

float LightweightMLPredictor::predict_fast(float speed, float engine_load, 
                                          common::TerrainType terrain) {
    // 超快速预测 - 使用简化的线性模型
    float terrain_factor = 1.0f;
    switch(terrain) {
        case common::TerrainType::FLAT: terrain_factor = 1.0f; break;
        case common::TerrainType::GENTLE_SLOPE: terrain_factor = 1.2f; break;
        case common::TerrainType::HILLY: terrain_factor = 1.5f; break;
        case common::TerrainType::STEEP_SLOPE: terrain_factor = 2.0f; break;
    }
    
    // 简化的负载计算
    float base_load = 0.2f + 0.015f * speed + 0.005f * engine_load;
    float prediction = base_load * terrain_factor;
    
    return std::max(0.0f, std::min(1.0f, prediction));
}

void LightweightMLPredictor::extract_features(const common::PerceptionData& perception_data,
                                             float current_cvt_ratio,
                                             float* features) const {
    features[0] = perception_data.vehicle_speed_kmh / 3.6f;  // 车速 (m/s)
    features[1] = perception_data.engine_load_percent;       // 发动机负载 (%)
    features[2] = perception_data.engine_speed_rpm;          // 发动机转速 (rpm)
    features[3] = perception_data.fuel_level_percent;        // 燃油消耗率估算
    features[4] = current_cvt_ratio;                         // CVT传动比
    features[5] = static_cast<float>(perception_data.terrain_type) / 3.0f;  // 地形因子
    features[6] = perception_data.coolant_temp_celsius;      // 冷却液温度
    features[7] = perception_data.load_factor;               // 历史负载因子
}

void LightweightMLPredictor::normalize_features(float* features) const {
    for(size_t i = 0; i < NUM_FEATURES; ++i) {
        // Z-score归一化
        features[i] = (features[i] - feature_stats_.mean[i]) / feature_stats_.std_dev[i];
        
        // 限制范围 [-3, 3]
        features[i] = std::max(-3.0f, std::min(3.0f, features[i]));
    }
}

float LightweightMLPredictor::calculate_confidence(const float* features) const {
    // 基于特征的置信度计算
    float confidence = 1.0f;
    
    // 检查特征是否在正常范围内
    for(size_t i = 0; i < NUM_FEATURES; ++i) {
        if(std::abs(features[i]) > 2.0f) {  // 超出2个标准差
            confidence *= 0.8f;
        }
    }
    
    // 基于历史预测误差调整置信度
    if(stats_.prediction_count > 10) {
        float error_factor = 1.0f - std::min(0.5f, stats_.average_error);
        confidence *= error_factor;
    }
    
    return std::max(0.1f, std::min(1.0f, confidence));
}

void LightweightMLPredictor::build_default_lookup_table() {
    // 构建基于经验的查找表
    size_t index = 0;
    
    // 不同地形和速度组合的预设值
    for(int terrain = 0; terrain < 4; ++terrain) {
        for(int speed_level = 0; speed_level < 8; ++speed_level) {
            for(int load_level = 0; load_level < 8; ++load_level) {
                if(index >= LOOKUP_TABLE_SIZE) break;
                
                auto& entry = lookup_model_.table[index];
                entry.speed_mps = speed_level * 3.75f;  // 0-30 m/s
                entry.engine_load = load_level * 12.5f; // 0-100%
                entry.terrain_code = terrain;
                
                // 基于经验的负载计算
                float base_load = 0.2f + 0.01f * entry.speed_mps + 0.003f * entry.engine_load;
                float terrain_multiplier = 1.0f + 0.3f * terrain;
                entry.predicted_load = base_load * terrain_multiplier;
                
                index++;
            }
        }
    }
    
    lookup_model_.entry_count = index;
}

void LightweightMLPredictor::build_default_decision_tree() {
    // 构建简单的决策树 (手工设计)
    tree_model_.node_count = 7;
    
    // 根节点: 基于车速分割
    tree_model_.nodes[0] = {0, 8.33f, 0.0f, 1, 2};  // speed < 30km/h
    
    // 低速分支
    tree_model_.nodes[1] = {1, 50.0f, 0.0f, 3, 4};  // engine_load < 50%
    tree_model_.nodes[3] = {5, 1.5f, 0.3f, 0, 0};   // 低速低负载
    tree_model_.nodes[4] = {5, 1.5f, 0.6f, 0, 0};   // 低速高负载
    
    // 高速分支  
    tree_model_.nodes[2] = {4, 2.0f, 0.0f, 5, 6};   // cvt_ratio < 2.0
    tree_model_.nodes[5] = {5, 1.5f, 0.4f, 0, 0};   // 高速低传动比
    tree_model_.nodes[6] = {5, 1.5f, 0.8f, 0, 0};   // 高速高传动比
}

float LightweightMLPredictor::LookupTableModel::predict(float speed, float engine_load, 
                                                       uint8_t terrain) const {
    if(entry_count == 0) return 0.5f;
    
    // 找到最近的表项
    float min_distance = 1e6f;
    float best_prediction = 0.5f;
    
    for(size_t i = 0; i < entry_count; ++i) {
        const auto& entry = table[i];
        if(entry.terrain_code != terrain) continue;
        
        float speed_diff = speed - entry.speed_mps;
        float load_diff = engine_load - entry.engine_load;
        float distance = speed_diff * speed_diff + load_diff * load_diff;
        
        if(distance < min_distance) {
            min_distance = distance;
            best_prediction = entry.predicted_load;
        }
    }
    
    return best_prediction;
}

float LightweightMLPredictor::DecisionTreeModel::predict(const float* features) const {
    if(node_count == 0) return 0.5f;
    
    size_t current_node = 0;
    
    while(current_node < node_count) {
        const auto& node = nodes[current_node];
        
        // 叶节点
        if(node.left_child == 0 && node.right_child == 0) {
            return node.value;
        }
        
        // 内部节点 - 根据特征值选择分支
        if(features[node.feature_index] < node.threshold) {
            current_node = node.left_child;
        } else {
            current_node = node.right_child;
        }
    }
    
    return 0.5f;  // 默认值
}

// 简化的时间戳函数 (实际实现需要平台相关代码)
uint32_t LightweightMLPredictor::get_timestamp_us() const {
    // 这里需要使用平台相关的高精度时间函数
    // 例如在NuttX上使用clock_gettime()
    return 0;  // 占位符
}

bool LightweightMLPredictor::update_parameters(const float* parameters, size_t param_count) {
    if(!is_initialized_ || param_count < NUM_FEATURES + 1) {
        return false;
    }
    
    // 更新线性模型参数
    for(size_t i = 0; i < NUM_FEATURES; ++i) {
        linear_model_.weights[i] = parameters[i];
    }
    linear_model_.bias = parameters[NUM_FEATURES];
    
    update_count_++;
    return true;
}

void LightweightMLPredictor::online_update(const float* features, float actual_load) {
    if(!is_initialized_) return;
    
    // 计算预测误差
    float prediction = linear_model_.predict(features);
    float error = actual_load - prediction;
    
    // 在线学习更新 (简单的梯度下降)
    linear_model_.update(features, error);
    
    // 更新统计信息
    float abs_error = std::abs(error);
    stats_.average_error = (stats_.average_error * (stats_.prediction_count - 1) + abs_error) / stats_.prediction_count;
    stats_.max_error = std::max(stats_.max_error, abs_error);
    
    // 更新历史记录
    update_history(prediction, actual_load);
}

void LightweightMLPredictor::update_history(float prediction, float actual_load) {
    history_[history_index_].prediction = prediction;
    history_[history_index_].actual_load = actual_load;
    history_[history_index_].timestamp_ms = 0;  // 需要实际时间戳
    
    history_index_ = (history_index_ + 1) % HISTORY_WINDOW_SIZE;
}

LightweightMLPredictor::ModelStats LightweightMLPredictor::get_statistics() const {
    return stats_;
}

float LightweightMLPredictor::get_confidence() const {
    return last_confidence_;
}

// HybridMLPredictor 实现
HybridMLPredictor::HybridMLPredictor() {
    remote_result_.is_valid = false;
    remote_result_.timestamp_ms = 0;
}

bool HybridMLPredictor::initialize() {
    return local_predictor_.initialize(LightweightMLPredictor::ModelType::HYBRID);
}

HybridMLPredictor::PredictionResult HybridMLPredictor::predict(
    const common::PerceptionData& perception_data, float current_cvt_ratio) {
    
    PredictionResult result;
    
    // 本地快速预测
    result.immediate_prediction = local_predictor_.predict_load(perception_data, current_cvt_ratio);
    
    // 检查是否有有效的远程预测结果
    if(remote_result_.is_valid && has_remote_update()) {
        result.optimized_prediction = fuse_predictions(
            result.immediate_prediction, remote_result_.prediction,
            local_predictor_.get_confidence(), remote_result_.confidence
        );
        result.confidence_score = std::max(local_predictor_.get_confidence(), remote_result_.confidence);
    } else {
        result.optimized_prediction = result.immediate_prediction;
        result.confidence_score = local_predictor_.get_confidence();
    }
    
    result.inference_time_us = 0;  // 需要实际时间计算
    return result;
}

void HybridMLPredictor::set_remote_prediction(float prediction, float confidence) {
    remote_result_.prediction = prediction;
    remote_result_.confidence = confidence;
    remote_result_.timestamp_ms = 0;  // 需要实际时间戳
    remote_result_.is_valid = true;
}

bool HybridMLPredictor::has_remote_update() const {
    if(!remote_result_.is_valid) return false;
    
    // 检查远程结果是否过期 (假设5秒过期)
    uint32_t current_time = 0;  // 需要实际时间戳
    return (current_time - remote_result_.timestamp_ms) < 5000;
}

float HybridMLPredictor::fuse_predictions(float local_pred, float remote_pred,
                                         float local_conf, float remote_conf) const {
    // 基于置信度的加权融合
    float total_conf = local_conf + remote_conf;
    if(total_conf < 0.01f) return local_pred;  // 避免除零
    
    float local_weight = local_conf / total_conf;
    float remote_weight = remote_conf / total_conf;
    
    return local_weight * local_pred + remote_weight * remote_pred;
}

} // namespace prediction
} // namespace vcu
