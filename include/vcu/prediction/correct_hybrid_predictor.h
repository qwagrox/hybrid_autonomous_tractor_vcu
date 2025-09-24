#ifndef CORRECT_HYBRID_PREDICTOR_H
#define CORRECT_HYBRID_PREDICTOR_H

#include "lightweight_ml_predictor.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/can/can_interface.h"
#include <atomic>
#include <memory>

namespace vcu {
namespace prediction {

/**
 * @class CorrectHybridPredictor
 * @brief 正确实现的混合部署预测器
 * 
 * 核心原则：
 * - VCU始终使用本地模型进行实时预测 (零延迟)
 * - 域控异步优化并下发模型参数 (提升精度)
 * - 参数更新不影响实时控制性能
 */
class CorrectHybridPredictor {
public:
    // 训练数据结构 (VCU → 域控)
    struct TrainingDataPoint {
        uint32_t timestamp_ms;
        common::PerceptionData sensor_data;
        float predicted_load;        // VCU的预测值
        float actual_load;          // 实际测量值
        float prediction_error;     // 预测误差
        float cvt_ratio;           // CVT传动比
        
        TrainingDataPoint() : timestamp_ms(0), predicted_load(0), 
                             actual_load(0), prediction_error(0), cvt_ratio(1.0f) {}
    };
    
    // 参数更新结构 (域控 → VCU)
    struct ParameterUpdate {
        uint32_t version;           // 参数版本号
        uint32_t timestamp_ms;      // 更新时间戳
        std::array<float, 8> weights;  // 模型权重
        float bias;                 // 偏置项
        float learning_rate;        // 建议学习率
        float confidence_score;     // 参数置信度
        uint32_t checksum;         // 校验和
        
        ParameterUpdate() : version(0), timestamp_ms(0), bias(0), 
                           learning_rate(0.001f), confidence_score(0), checksum(0) {
            weights.fill(0.0f);
        }
        
        uint32_t calculate_checksum() const;
        bool validate_checksum() const;
    };
    
    /**
     * @brief 构造函数
     */
    CorrectHybridPredictor(PlatformInterface* platform,
                          std::shared_ptr<can::ICanInterface> can_interface);
    
    /**
     * @brief 析构函数
     */
    ~CorrectHybridPredictor();
    
    /**
     * @brief 初始化预测器
     */
    bool initialize();
    
    /**
     * @brief 关闭预测器
     */
    void shutdown();
    
    /**
     * @brief 实时负载预测 (主接口 - 零延迟)
     * @param perception_data 感知数据
     * @param cvt_ratio 当前CVT传动比
     * @return 预测负载 (0.0-1.0)
     */
    float predict_load_realtime(const common::PerceptionData& perception_data,
                               float cvt_ratio);
    
    /**
     * @brief 设置实际负载反馈 (用于训练数据收集)
     * @param actual_load 实际测量的负载
     */
    void set_actual_load_feedback(float actual_load);
    
    /**
     * @brief 获取预测统计信息
     */
    struct PredictionStats {
        uint32_t total_predictions;
        float average_error;
        float current_accuracy;
        uint32_t parameter_updates_received;
        uint32_t training_data_sent;
        uint32_t average_inference_time_us;
    };
    
    PredictionStats get_statistics() const;
    
    /**
     * @brief 获取当前模型参数版本
     */
    uint32_t get_current_parameter_version() const;

private:
    // 本地预测器 (始终可用)
    class LocalRealtimePredictor {
    public:
        LocalRealtimePredictor();
        
        // 实时预测 (目标 <100μs)
        float predict(const common::PerceptionData& data, float cvt_ratio);
        
        // 应用新参数 (非阻塞)
        bool apply_parameter_update(const ParameterUpdate& update);
        
        // 获取当前参数
        ParameterUpdate get_current_parameters() const;
        
        // 性能统计
        uint32_t get_last_inference_time_us() const;
        
    private:
        // 轻量级线性模型
        struct LinearModel {
            std::array<float, 8> weights;
            float bias;
            uint32_t version;
            
            LinearModel() : bias(0.2f), version(0) {
                // 初始化为经验权重
                weights = {0.02f, 0.008f, -0.0001f, 0.005f, 0.15f, 0.3f, 0.001f, 0.1f};
            }
            
            float predict(const float* features) const {
                float result = bias;
                for(size_t i = 0; i < weights.size(); ++i) {
                    result += weights[i] * features[i];
                }
                return std::clamp(result, 0.0f, 1.0f);
            }
        };
        
        LinearModel current_model_;
        std::atomic<uint32_t> last_inference_time_us_;
        
        // 特征提取
        void extract_features(const common::PerceptionData& data, 
                             float cvt_ratio, float* features) const;
    };
    
    // 训练数据管理器
    class TrainingDataManager {
    public:
        TrainingDataManager();
        
        // 添加训练数据点
        void add_data_point(const TrainingDataPoint& data_point);
        
        // 获取待发送的数据批次
        std::vector<TrainingDataPoint> get_batch_for_transmission();
        
        // 检查是否需要发送数据
        bool should_send_data() const;
        
        // 获取统计信息
        uint32_t get_data_points_collected() const;
        uint32_t get_batches_sent() const;
        
    private:
        static constexpr size_t BUFFER_SIZE = 100;
        static constexpr size_t BATCH_SIZE = 20;
        static constexpr uint32_t SEND_INTERVAL_MS = 5000;  // 5秒发送一次
        
        std::array<TrainingDataPoint, BUFFER_SIZE> data_buffer_;
        size_t buffer_index_;
        size_t data_count_;
        uint32_t last_send_time_ms_;
        uint32_t batches_sent_;
    };
    
    // 参数更新管理器
    class ParameterUpdateManager {
    public:
        ParameterUpdateManager();
        
        // 处理接收到的参数更新
        bool process_parameter_update(const ParameterUpdate& update);
        
        // 检查是否有待应用的更新
        bool has_pending_update() const;
        
        // 获取待应用的更新
        std::optional<ParameterUpdate> get_pending_update();
        
        // 获取统计信息
        uint32_t get_updates_received() const;
        uint32_t get_updates_applied() const;
        
    private:
        std::optional<ParameterUpdate> pending_update_;
        std::atomic<bool> has_pending_;
        uint32_t updates_received_;
        uint32_t updates_applied_;
        uint32_t current_version_;
        
        // 参数验证
        bool validate_parameter_update(const ParameterUpdate& update) const;
    };
    
    // CAN通信管理器
    class CANCommunicationManager {
    public:
        CANCommunicationManager(std::shared_ptr<can::ICanInterface> can_interface);
        
        // 启动通信任务
        bool start_communication();
        
        // 停止通信任务
        void stop_communication();
        
        // 发送训练数据
        bool send_training_data(const std::vector<TrainingDataPoint>& data_batch);
        
        // 设置参数更新回调
        void set_parameter_update_callback(
            std::function<void(const ParameterUpdate&)> callback);
        
    private:
        std::shared_ptr<can::ICanInterface> can_interface_;
        std::unique_ptr<ThreadInterface> communication_thread_;
        std::atomic<bool> is_running_;
        
        std::function<void(const ParameterUpdate&)> parameter_update_callback_;
        
        // CAN消息处理
        void communication_task();
        void process_incoming_can_messages();
        void send_training_data_via_can(const std::vector<TrainingDataPoint>& data);
        
        // 协议定义
        static constexpr uint32_t CAN_ID_TRAINING_DATA = 0x300;
        static constexpr uint32_t CAN_ID_PARAMETER_UPDATE = 0x301;
        
        // 消息编解码
        std::vector<can::CanFrame> encode_training_data(const std::vector<TrainingDataPoint>& data);
        std::optional<ParameterUpdate> decode_parameter_update(const can::CanFrame& frame);
    };
    
    // 性能监控器
    class PerformanceMonitor {
    public:
        PerformanceMonitor();
        
        // 记录预测性能
        void record_prediction(float prediction, float actual_load, uint32_t inference_time_us);
        
        // 记录参数更新
        void record_parameter_update();
        
        // 记录数据发送
        void record_data_transmission();
        
        // 获取统计信息
        PredictionStats get_statistics() const;
        
        // 获取当前精度
        float get_current_accuracy() const;
        
    private:
        struct Statistics {
            std::atomic<uint32_t> total_predictions;
            std::atomic<uint32_t> parameter_updates;
            std::atomic<uint32_t> data_transmissions;
            std::atomic<uint32_t> total_inference_time_us;
            
            Statistics() : total_predictions(0), parameter_updates(0), 
                          data_transmissions(0), total_inference_time_us(0) {}
        } stats_;
        
        // 滑动窗口精度计算
        static constexpr size_t ACCURACY_WINDOW_SIZE = 50;
        std::array<float, ACCURACY_WINDOW_SIZE> recent_errors_;
        size_t error_index_;
        float current_accuracy_;
        
        void update_accuracy(float error);
    };
    
    // 主控制循环
    void main_prediction_loop();
    
    // 后台任务
    void background_tasks();
    void parameter_update_task();
    void data_transmission_task();
    
    // 成员变量
    PlatformInterface* platform_;
    std::shared_ptr<can::ICanInterface> can_interface_;
    
    // 核心组件
    std::unique_ptr<LocalRealtimePredictor> local_predictor_;
    std::unique_ptr<TrainingDataManager> training_data_manager_;
    std::unique_ptr<ParameterUpdateManager> parameter_update_manager_;
    std::unique_ptr<CANCommunicationManager> can_comm_manager_;
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
    } recent_prediction_;
    
    std::atomic<bool> has_recent_prediction_;
};

} // namespace prediction
} // namespace vcu

#endif // CORRECT_HYBRID_PREDICTOR_H
