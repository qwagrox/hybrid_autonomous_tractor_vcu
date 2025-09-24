#ifndef KNOWLEDGE_DISTILLATION_PREDICTOR_H
#define KNOWLEDGE_DISTILLATION_PREDICTOR_H

#include "vcu/common/vcu_data_types.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/can/can_interface.h"
#include <array>
#include <memory>

namespace vcu {
namespace prediction {

/**
 * @class KnowledgeDistillationPredictor
 * @brief 基于知识蒸馏的负载预测器
 * 
 * 核心思想：
 * - 域控运行复杂"教师模型"(CNN/RNN/Transformer)
 * - VCU运行简单"学生模型"(线性回归+特征工程)
 * - 教师模型"教会"学生模型如何预测
 * - 学生模型获得复杂模型的预测能力，但保持简单快速
 */
class KnowledgeDistillationPredictor {
public:
    // 蒸馏后的学生模型参数
    struct DistilledModelParameters {
        uint32_t version;                    // 参数版本
        uint32_t timestamp_ms;               // 更新时间戳
        
        // 线性模型权重 (对应工程化特征)
        std::array<float, 16> feature_weights;
        float bias;
        
        // 特征工程参数 (由教师模型优化发现)
        struct FeatureEngineering {
            // 非线性变换参数
            float speed_power;               // 速度的幂次 (如1.2, 1.5等)
            float load_power;                // 负载的幂次
            
            // 交互特征权重
            float speed_load_interaction;    // 速度-负载交互强度
            float speed_terrain_interaction; // 速度-地形交互强度
            float load_terrain_interaction;  // 负载-地形交互强度
            
            // 动态特征参数
            float momentum_decay;            // 动量衰减因子
            float acceleration_sensitivity;  // 加速度敏感性
            float history_weight;           // 历史权重
            
            // 上下文调整参数
            float temperature_sensitivity;   // 温度敏感性
            float altitude_sensitivity;      // 海拔敏感性
            float weather_sensitivity;       // 天气敏感性
            
        } feature_params;
        
        // 置信度和质量指标
        float distillation_quality;         // 蒸馏质量 (0-1)
        float teacher_accuracy;              // 教师模型精度
        float student_accuracy;              // 学生模型精度
        
        uint32_t checksum;                   // 校验和
        
        DistilledModelParameters() : version(0), timestamp_ms(0), bias(0.0f),
                                   distillation_quality(0.0f), teacher_accuracy(0.0f),
                                   student_accuracy(0.0f), checksum(0) {
            feature_weights.fill(0.0f);
        }
        
        uint32_t calculate_checksum() const;
        bool validate_checksum() const;
    };
    
    // 教师模型的训练数据 (VCU → 域控)
    struct TeacherTrainingData {
        uint32_t timestamp_ms;
        common::PerceptionData sensor_data;
        float actual_load;                   // 真实负载
        float current_prediction;            // 当前学生模型预测
        float prediction_confidence;         // 预测置信度
        
        // 扩展环境信息
        float ambient_temperature;           // 环境温度
        float altitude_m;                   // 海拔高度
        uint8_t weather_condition;          // 天气状况
        float road_grade_percent;           // 道路坡度
        
        TeacherTrainingData() : timestamp_ms(0), actual_load(0.0f),
                               current_prediction(0.0f), prediction_confidence(0.0f),
                               ambient_temperature(25.0f), altitude_m(0.0f),
                               weather_condition(0), road_grade_percent(0.0f) {}
    };
    
    /**
     * @brief 构造函数
     */
    KnowledgeDistillationPredictor(PlatformInterface* platform,
                                  std::shared_ptr<can::ICanInterface> can_interface);
    
    /**
     * @brief 析构函数
     */
    ~KnowledgeDistillationPredictor();
    
    /**
     * @brief 初始化预测器
     */
    bool initialize();
    
    /**
     * @brief 关闭预测器
     */
    void shutdown();
    
    /**
     * @brief 实时负载预测 (学生模型)
     * @param perception_data 感知数据
     * @param cvt_ratio 当前CVT传动比
     * @return 预测负载
     */
    float predict_load_student_model(const common::PerceptionData& perception_data,
                                    float cvt_ratio);
    
    /**
     * @brief 设置实际负载反馈
     * @param actual_load 实际负载
     */
    void set_actual_load_feedback(float actual_load);
    
    /**
     * @brief 更新蒸馏模型参数
     * @param params 新的蒸馏参数
     * @return 更新结果
     */
    bool update_distilled_parameters(const DistilledModelParameters& params);
    
    /**
     * @brief 获取当前模型性能
     */
    struct ModelPerformance {
        float current_accuracy;              // 当前精度
        float average_inference_time_us;     // 平均推理时间
        uint32_t total_predictions;          // 总预测次数
        uint32_t parameter_updates;          // 参数更新次数
        float distillation_effectiveness;    // 蒸馏有效性
    };
    
    ModelPerformance get_performance_metrics() const;

private:
    // 学生模型 (VCU本地运行)
    class StudentModel {
    public:
        StudentModel();
        
        // 快速预测 (目标 <50μs)
        float predict(const common::PerceptionData& data, float cvt_ratio);
        
        // 更新蒸馏参数
        bool update_parameters(const DistilledModelParameters& params);
        
        // 获取当前参数版本
        uint32_t get_parameter_version() const;
        
        // 性能统计
        uint32_t get_last_inference_time_us() const;
        
    private:
        DistilledModelParameters current_params_;
        
        // 特征工程 (基于教师模型优化)
        std::array<float, 16> engineer_features(const common::PerceptionData& data,
                                               float cvt_ratio) const;
        
        // 基础特征
        void extract_basic_features(const common::PerceptionData& data,
                                   float cvt_ratio, float* features) const;
        
        // 交互特征 (教师模型发现的有效组合)
        void extract_interaction_features(const common::PerceptionData& data,
                                        float cvt_ratio, float* features) const;
        
        // 动态特征 (时序相关)
        void extract_dynamic_features(const common::PerceptionData& data,
                                     float* features) const;
        
        // 上下文特征 (环境相关)
        void extract_contextual_features(const common::PerceptionData& data,
                                       float* features) const;
        
        // 历史数据缓存 (用于动态特征计算)
        struct HistoryBuffer {
            static constexpr size_t BUFFER_SIZE = 10;
            std::array<float, BUFFER_SIZE> speed_history;
            std::array<float, BUFFER_SIZE> load_history;
            size_t index;
            
            HistoryBuffer() : index(0) {
                speed_history.fill(0.0f);
                load_history.fill(0.0f);
            }
            
            void add_sample(float speed, float load) {
                speed_history[index] = speed;
                load_history[index] = load;
                index = (index + 1) % BUFFER_SIZE;
            }
            
            float calculate_momentum() const;
            float calculate_acceleration() const;
        } history_buffer_;
    };
    
    // 教师模型通信管理器
    class TeacherModelCommunicator {
    public:
        TeacherModelCommunicator(std::shared_ptr<can::ICanInterface> can_interface);
        
        // 启动通信
        bool start_communication();
        
        // 停止通信
        void stop_communication();
        
        // 发送训练数据到教师模型
        bool send_training_data(const std::vector<TeacherTrainingData>& data);
        
        // 设置参数更新回调
        void set_parameter_update_callback(
            std::function<void(const DistilledModelParameters&)> callback);
        
        // 获取通信统计
        struct CommunicationStats {
            uint32_t data_packets_sent;
            uint32_t parameter_updates_received;
            uint32_t communication_errors;
            float average_latency_ms;
        };
        
        CommunicationStats get_communication_stats() const;
        
    private:
        std::shared_ptr<can::ICanInterface> can_interface_;
        std::unique_ptr<ThreadInterface> communication_thread_;
        std::atomic<bool> is_running_;
        
        std::function<void(const DistilledModelParameters&)> parameter_callback_;
        
        // 通信协议
        static constexpr uint32_t CAN_ID_TEACHER_TRAINING_DATA = 0x400;
        static constexpr uint32_t CAN_ID_DISTILLED_PARAMETERS = 0x401;
        
        // 消息处理
        void communication_task();
        void process_incoming_parameters();
        
        // 编解码
        std::vector<can::CanFrame> encode_training_data(const std::vector<TeacherTrainingData>& data);
        std::optional<DistilledModelParameters> decode_distilled_parameters(const std::vector<can::CanFrame>& frames);
        
        // 统计信息
        CommunicationStats comm_stats_;
    };
    
    // 训练数据收集器
    class TrainingDataCollector {
    public:
        TrainingDataCollector();
        
        // 添加训练样本
        void add_training_sample(const common::PerceptionData& sensor_data,
                               float predicted_load,
                               float actual_load,
                               float confidence);
        
        // 获取待发送的训练批次
        std::vector<TeacherTrainingData> get_training_batch();
        
        // 检查是否需要发送数据
        bool should_send_training_data() const;
        
        // 获取收集统计
        uint32_t get_samples_collected() const;
        
    private:
        static constexpr size_t BUFFER_SIZE = 200;
        static constexpr size_t BATCH_SIZE = 50;
        static constexpr uint32_t SEND_INTERVAL_MS = 10000;  // 10秒发送一次
        
        std::array<TeacherTrainingData, BUFFER_SIZE> data_buffer_;
        size_t buffer_index_;
        size_t sample_count_;
        uint32_t last_send_time_ms_;
        
        // 数据质量过滤
        bool is_valid_training_sample(const TeacherTrainingData& sample) const;
    };
    
    // 性能监控器
    class PerformanceMonitor {
    public:
        PerformanceMonitor();
        
        // 记录预测性能
        void record_prediction(float prediction, float actual_load, 
                             uint32_t inference_time_us);
        
        // 记录参数更新
        void record_parameter_update(const DistilledModelParameters& params);
        
        // 获取性能指标
        ModelPerformance get_performance_metrics() const;
        
        // 评估蒸馏有效性
        float evaluate_distillation_effectiveness() const;
        
    private:
        struct PerformanceData {
            std::atomic<uint32_t> total_predictions;
            std::atomic<uint32_t> parameter_updates;
            std::atomic<uint32_t> total_inference_time_us;
            
            PerformanceData() : total_predictions(0), parameter_updates(0),
                               total_inference_time_us(0) {}
        } perf_data_;
        
        // 精度计算 (滑动窗口)
        static constexpr size_t ACCURACY_WINDOW = 100;
        std::array<float, ACCURACY_WINDOW> recent_errors_;
        size_t error_index_;
        
        // 蒸馏质量跟踪
        float last_distillation_quality_;
        float baseline_accuracy_;  // 基准精度 (用于对比)
        
        void update_accuracy_metrics(float error);
    };
    
    // 成员变量
    PlatformInterface* platform_;
    std::shared_ptr<can::ICanInterface> can_interface_;
    
    // 核心组件
    std::unique_ptr<StudentModel> student_model_;
    std::unique_ptr<TeacherModelCommunicator> teacher_communicator_;
    std::unique_ptr<TrainingDataCollector> data_collector_;
    std::unique_ptr<PerformanceMonitor> performance_monitor_;
    
    // 平台抽象接口
    std::unique_ptr<ThreadInterface> background_thread_;
    std::unique_ptr<MutexInterface> data_mutex_;
    std::unique_ptr<TimeInterface> time_interface_;
    
    // 当前状态
    std::atomic<bool> is_initialized_;
    std::atomic<bool> is_running_;
    
    // 最近的预测数据 (用于训练数据收集)
    struct RecentPrediction {
        common::PerceptionData sensor_data;
        float predicted_load;
        float cvt_ratio;
        uint32_t timestamp_ms;
        bool has_feedback;
        float confidence;
    } recent_prediction_;
    
    std::atomic<bool> has_recent_prediction_;
    
    // 后台任务
    void background_tasks();
    void data_collection_task();
    void parameter_update_task();
    void performance_monitoring_task();
};

} // namespace prediction
} // namespace vcu

#endif // KNOWLEDGE_DISTILLATION_PREDICTOR_H
