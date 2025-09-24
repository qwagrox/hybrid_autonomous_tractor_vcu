// Copyright 2025 Manus AI

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
            // 混合预测：结合多个模型的结果
            float linear_pred = linear_model_.predict(features);
            float table_pred = lookup_model_.predict(
                perception_data.vehicle_speed_kmh / 3.6f,
                perception_data.engine_load_percent,
                static_cast<uint8_t>(perception_data.terrain_type)
            );
            float tree_pred = tree_model_.predict(features);
            
            // 加权平均 (可以根据历史性能调整权重)
            prediction = 0.5f * linear_pred + 0.3f * table_pred + 0.2f * tree_pred;
            break;
    }
    
    // 限制预测范围
    prediction = std::max(0.0f, std::min(1.0f, prediction));
    
    // 计算置信度
    last_confidence_ = calculate_confidence(features);
    
    // 更新统计信息
    uint32_t inference_time = get_timestamp_us() - start_time;
    stats_.inference_time_us = inference_time;
    stats_.prediction_count++;
    
    return prediction;
}

float LightweightMLPredictor::predict_fast(float speed, float engine_load, common::TerrainType terrain) {
    if(!is_initialized_) {
        return 0.5f;
    }
    
    // 超快速预测：只使用查找表
    return lookup_model_.predict(speed, engine_load, static_cast<uint8_t>(terrain));
}

bool LightweightMLPredictor::update_parameters(const float* parameters, size_t param_count) {
    if(!parameters || param_count != NUM_FEATURES + 1) {  // weights + bias
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
    if(!features) return;
    
    // 使用当前模型进行预测
    float prediction = 0.0f;
    switch(current_model_type_) {
        case ModelType::LINEAR_REGRESSION:
        case ModelType::HYBRID:
            prediction = linear_model_.predict(features);
            break;
        default:
            return;  // 其他模型暂不支持在线学习
    }
    
    // 计算误差并更新模型
    float error = actual_load - prediction;
    linear_model_.update(features, error);
    
    // 更新历史记录
    update_history(prediction, actual_load);
    
    // 更新统计信息
    float abs_error = std::abs(error);
    stats_.average_error = (stats_.average_error * (stats_.prediction_count - 1) + abs_error) / stats_.prediction_count;
    stats_.max_error = std::max(stats_.max_error, abs_error);
}

float LightweightMLPredictor::get_confidence() const {
    return last_confidence_;
}

LightweightMLPredictor::ModelStats LightweightMLPredictor::get_statistics() const {
    return stats_;
}

// 私有方法实现

void LightweightMLPredictor::extract_features(const common::PerceptionData& perception_data,
                                            float current_cvt_ratio,
                                            float* features) const {
    features[0] = perception_data.vehicle_speed_kmh / 3.6f;  // 车速 (m/s)
    features[1] = perception_data.engine_load_percent;       // 发动机负载 (%)
    features[2] = perception_data.engine_rpm;               // 发动机转速 (rpm)
    features[3] = perception_data.fuel_consumption_lph;     // 燃油消耗 (L/h)
    features[4] = current_cvt_ratio;                        // CVT传动比
    features[5] = static_cast<float>(perception_data.terrain_type) / 4.0f;  // 地形因子 (归一化)
    features[6] = perception_data.ambient_temperature_c;    // 环境温度 (°C)
    
    // 历史负载平均值
    float history_avg = 0.0f;
    size_t valid_count = 0;
    for(const auto& entry : history_) {
        if(entry.timestamp_ms > 0) {
            history_avg += entry.actual_load;
            valid_count++;
        }
    }
    features[7] = valid_count > 0 ? history_avg / valid_count : 0.5f;
}

void LightweightMLPredictor::normalize_features(float* features) const {
    for(size_t i = 0; i < NUM_FEATURES; ++i) {
        // Z-score 归一化
        features[i] = (features[i] - feature_stats_.mean[i]) / feature_stats_.std_dev[i];
        
        // 限制范围 [-3, 3]
        features[i] = std::max(-3.0f, std::min(3.0f, features[i]));
    }
}

void LightweightMLPredictor::update_history(float prediction, float actual_load) {
    history_[history_index_].prediction = prediction;
    history_[history_index_].actual_load = actual_load;
    history_[history_index_].timestamp_ms = get_timestamp_ms();
    
    history_index_ = (history_index_ + 1) % HISTORY_WINDOW_SIZE;
}

float LightweightMLPredictor::calculate_confidence(const float* features) const {
    // 基于特征的置信度计算
    float confidence = 1.0f;
    
    // 检查特征是否在正常范围内
    for(size_t i = 0; i < NUM_FEATURES; ++i) {
        if(std::abs(features[i]) > 2.0f) {  // 超出2个标准差
            confidence *= 0.8f;  // 降低置信度
        }
    }
    
    // 基于历史预测精度
    if(stats_.prediction_count > 10) {
        float accuracy = 1.0f - std::min(1.0f, stats_.average_error);
        confidence *= accuracy;
    }
    
    return std::max(0.1f, std::min(1.0f, confidence));
}

// 查找表模型实现
float LightweightMLPredictor::LookupTableModel::predict(float speed, float engine_load, uint8_t terrain) const {
    if(entry_count == 0) {
        return 0.5f;  // 默认值
    }
    
    // 寻找最近邻
    float min_distance = std::numeric_limits<float>::max();
    size_t best_index = 0;
    
    for(size_t i = 0; i < entry_count; ++i) {
        float speed_diff = speed - table[i].speed_mps;
        float load_diff = engine_load - table[i].engine_load;
        float terrain_diff = static_cast<float>(terrain) - static_cast<float>(table[i].terrain_code);
        
        float distance = speed_diff * speed_diff + 
                        (load_diff * load_diff) * 0.01f +  // 缩放负载差异
                        terrain_diff * terrain_diff * 0.25f;  // 缩放地形差异
        
        if(distance < min_distance) {
            min_distance = distance;
            best_index = i;
        }
    }
    
    return table[best_index].predicted_load;
}

void LightweightMLPredictor::LookupTableModel::build_table(const float* training_data, size_t data_count) {
    // 简化的查找表构建 (实际应该使用聚类算法)
    entry_count = std::min(data_count, LOOKUP_TABLE_SIZE);
    
    for(size_t i = 0; i < entry_count; ++i) {
        size_t data_index = i * 4;  // 假设每个样本4个值: speed, load, terrain, target
        table[i].speed_mps = training_data[data_index];
        table[i].engine_load = training_data[data_index + 1];
        table[i].terrain_code = static_cast<uint8_t>(training_data[data_index + 2]);
        table[i].predicted_load = training_data[data_index + 3];
    }
}

// 决策树模型实现
float LightweightMLPredictor::DecisionTreeModel::predict(const float* features) const {
    if(node_count == 0) {
        return 0.5f;  // 默认值
    }
    
    uint8_t current_node = 0;  // 从根节点开始
    
    while(current_node < node_count) {
        const Node& node = nodes[current_node];
        
        if(node.left_child == 0 && node.right_child == 0) {
            // 叶节点
            return node.value;
        }
        
        // 内部节点：根据特征值选择子节点
        if(features[node.feature_index] <= node.threshold) {
            current_node = node.left_child;
        } else {
            current_node = node.right_child;
        }
        
        // 防止无限循环
        if(current_node == 0) break;
    }
    
    return 0.5f;  // 默认值
}

void LightweightMLPredictor::DecisionTreeModel::build_tree(const float* training_data, const float* labels, size_t data_count) {
    // 简化的决策树构建 (实际应该使用ID3或C4.5算法)
    if(data_count == 0) return;
    
    // 创建简单的两层决策树
    node_count = 3;
    
    // 根节点：基于车速分割
    nodes[0].feature_index = 0;  // 车速特征
    nodes[0].threshold = 0.0f;   // 归一化后的阈值
    nodes[0].left_child = 1;
    nodes[0].right_child = 2;
    
    // 左子节点：低速情况
    nodes[1].feature_index = 0;
    nodes[1].threshold = 0.0f;
    nodes[1].value = 0.3f;  // 低负载
    nodes[1].left_child = 0;
    nodes[1].right_child = 0;
    
    // 右子节点：高速情况
    nodes[2].feature_index = 0;
    nodes[2].threshold = 0.0f;
    nodes[2].value = 0.7f;  // 高负载
    nodes[2].left_child = 0;
    nodes[2].right_child = 0;
}

// 辅助函数
uint32_t get_timestamp_us() {
    // 简化实现，实际应该使用平台相关的高精度时间函数
    return 0;
}

uint32_t get_timestamp_ms() {
    // 简化实现，实际应该使用平台相关的时间函数
    return 0;
}

void LightweightMLPredictor::build_default_lookup_table() {
    // 构建默认查找表 (基于经验数据)
    lookup_model_.entry_count = 16;
    
    // 填充一些典型的工况数据
    lookup_model_.table[0] = {5.0f, 20.0f, 0, 0.2f};   // 低速平地
    lookup_model_.table[1] = {5.0f, 40.0f, 1, 0.4f};   // 低速上坡
    lookup_model_.table[2] = {10.0f, 30.0f, 0, 0.3f};  // 中速平地
    lookup_model_.table[3] = {10.0f, 60.0f, 1, 0.6f};  // 中速上坡
    lookup_model_.table[4] = {15.0f, 40.0f, 0, 0.4f};  // 高速平地
    lookup_model_.table[5] = {15.0f, 80.0f, 1, 0.8f};  // 高速上坡
    lookup_model_.table[6] = {20.0f, 50.0f, 0, 0.5f};  // 很高速平地
    lookup_model_.table[7] = {20.0f, 90.0f, 1, 0.9f};  // 很高速上坡
    lookup_model_.table[8] = {8.0f, 25.0f, 2, 0.3f};   // 中速泥地
    lookup_model_.table[9] = {12.0f, 45.0f, 2, 0.5f};  // 高速泥地
    lookup_model_.table[10] = {6.0f, 35.0f, 3, 0.4f};  // 低速沙地
    lookup_model_.table[11] = {10.0f, 55.0f, 3, 0.6f}; // 中速沙地
    lookup_model_.table[12] = {3.0f, 15.0f, 0, 0.15f}; // 极低速平地
    lookup_model_.table[13] = {25.0f, 60.0f, 0, 0.6f}; // 极高速平地
    lookup_model_.table[14] = {7.0f, 70.0f, 1, 0.7f};  // 低速陡坡
    lookup_model_.table[15] = {18.0f, 85.0f, 1, 0.85f}; // 高速陡坡
}

void LightweightMLPredictor::build_default_decision_tree() {
    // 构建默认决策树
    tree_model_.node_count = 7;
    
    // 根节点：基于车速
    tree_model_.nodes[0] = {0, 0.0f, 0.0f, 1, 2};  // 特征0(车速), 阈值0, 左子1, 右子2
    
    // 左子树：低速分支
    tree_model_.nodes[1] = {1, 0.0f, 0.0f, 3, 4};  // 特征1(负载), 左子3, 右子4
    tree_model_.nodes[3] = {0, 0.0f, 0.25f, 0, 0}; // 叶节点：低速低负载
    tree_model_.nodes[4] = {0, 0.0f, 0.45f, 0, 0}; // 叶节点：低速高负载
    
    // 右子树：高速分支
    tree_model_.nodes[2] = {1, 0.0f, 0.0f, 5, 6};  // 特征1(负载), 左子5, 右子6
    tree_model_.nodes[5] = {0, 0.0f, 0.55f, 0, 0}; // 叶节点：高速低负载
    tree_model_.nodes[6] = {0, 0.0f, 0.75f, 0, 0}; // 叶节点：高速高负载
}

// HybridMLPredictor 实现
HybridMLPredictor::HybridMLPredictor() {
    remote_result_.is_valid = false;
    remote_result_.timestamp_ms = 0;
}

bool HybridMLPredictor::initialize() {
    // 直接初始化，不检查返回值，因为它总是返回true
    local_predictor_.initialize(LightweightMLPredictor::ModelType::HYBRID);
    return true;
}

HybridMLPredictor::PredictionResult HybridMLPredictor::predict(
    const common::PerceptionData& perception_data, float current_cvt_ratio) {
    
    PredictionResult result;
    // 移除未使用的start_time变量
    
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
    // 修复条件判断问题 - 使用实际的时间戳比较
    // 如果没有实际时间戳，简化逻辑
    return remote_result_.is_valid && (remote_result_.timestamp_ms > 0);
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
