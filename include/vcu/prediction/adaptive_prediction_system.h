#ifndef ADAPTIVE_PREDICTION_SYSTEM_H
#define ADAPTIVE_PREDICTION_SYSTEM_H

#include "lightweight_ml_predictor.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/can/can_interface.h"
#include <array>
#include <optional>
#include <functional>

namespace vcu {
namespace prediction {

/**
 * @class AdaptivePredictionSystem
 * @brief 完全自适应的负载预测系统
 * 
 * 特点：
 * - 多时间尺度自适应 (毫秒/秒/分钟级)
 * - 预测性参数缓存
 * - 智能通信调度
 * - 动态资源管理
 */
class AdaptivePredictionSystem {
public:
    // 驾驶环境上下文
    struct DrivingContext {
        float current_speed_mps;
        float acceleration_trend;      // 加速度趋势
        float upcoming_slope;          // 前方坡度 (基于GPS+地图)
        uint32_t traffic_density;      // 交通密度
        uint32_t route_complexity;     // 路线复杂度
        bool is_highway;              // 是否高速公路
        
        DrivingContext() : current_speed_mps(0), acceleration_trend(0), 
                          upcoming_slope(0), traffic_density(0), 
                          route_complexity(0), is_highway(false) {}
    };
    
    // 系统负载状态
    struct SystemLoadState {
        float vcu_cpu_usage;          // VCU CPU使用率
        float vcu_memory_usage;       // VCU内存使用率
        float domain_controller_load; // 域控负载
        float network_latency_ms;     // 网络延迟
        bool cloud_available;         // 云端可用性
        
        SystemLoadState() : vcu_cpu_usage(0), vcu_memory_usage(0),
                           domain_controller_load(0), network_latency_ms(0),
                           cloud_available(false) {}
    };
    
    // 自适应策略
    struct AdaptationStrategy {
        enum ModelComplexity { SIMPLE, MEDIUM, COMPLEX };
        enum UpdateFrequency { LOW, MEDIUM, HIGH };
        enum CommunicationMode { LOCAL_ONLY, HYBRID, CLOUD_ASSISTED };
        
        ModelComplexity model_complexity;
        UpdateFrequency parameter_update_freq;
        CommunicationMode communication_mode;
        float learning_rate_multiplier;
        bool enable_predictive_caching;
        
        AdaptationStrategy() : model_complexity(MEDIUM), 
                              parameter_update_freq(MEDIUM),
                              communication_mode(HYBRID),
                              learning_rate_multiplier(1.0f),
                              enable_predictive_caching(true) {}
    };
    
    /**
     * @brief 构造函数
     */
    AdaptivePredictionSystem(PlatformInterface* platform,
                            std::shared_ptr<can::ICanInterface> can_interface);
    
    /**
     * @brief 析构函数
     */
    ~AdaptivePredictionSystem();
    
    /**
     * @brief 初始化系统
     */
    bool initialize();
    
    /**
     * @brief 关闭系统
     */
    void shutdown();
    
    /**
     * @brief 自适应负载预测 (主接口)
     * @param perception_data 感知数据
     * @param cvt_ratio 当前CVT传动比
     * @param context 驾驶环境上下文
     * @return 预测结果
     */
    struct PredictionResult {
        float predicted_load;         // 预测负载
        float confidence_score;       // 置信度
        uint32_t inference_time_us;   // 推理时间
        AdaptationStrategy strategy;  // 使用的策略
    };
    
    PredictionResult predict_adaptive(const common::PerceptionData& perception_data,
                                     float cvt_ratio,
                                     const DrivingContext& context);
    
    /**
     * @brief 更新驾驶环境上下文
     */
    void update_driving_context(const DrivingContext& context);
    
    /**
     * @brief 更新系统负载状态
     */
    void update_system_load_state(const SystemLoadState& load_state);
    
    /**
     * @brief 设置实际负载反馈 (用于在线学习)
     */
    void set_actual_load_feedback(float actual_load);
    
    /**
     * @brief 获取当前自适应策略
     */
    AdaptationStrategy get_current_strategy() const;
    
    /**
     * @brief 获取系统性能统计
     */
    struct PerformanceStats {
        uint32_t total_predictions;
        float average_accuracy;
        float average_inference_time_us;
        uint32_t cache_hit_rate;
        uint32_t adaptation_count;
        float system_efficiency_score;
    };
    
    PerformanceStats get_performance_stats() const;

private:
    // 本地自适应预测器
    class LocalAdaptivePredictor {
    public:
        LocalAdaptivePredictor();
        
        float predict_with_local_adaptation(const common::PerceptionData& data,
                                           float cvt_ratio,
                                           const DrivingContext& context);
        
        void update_with_feedback(float prediction, float actual_load);
        void set_adaptation_parameters(const AdaptationStrategy& strategy);
        
    private:
        LightweightMLPredictor base_predictor_;
        
        // 实时误差统计
        struct ErrorTracker {
            std::array<float, 50> recent_errors;
            size_t error_index;
            float mean_error;
            float error_variance;
            
            void add_error(float error);
            float get_bias_correction() const;
        } error_tracker_;
        
        // 环境自适应参数
        struct ContextualParameters {
            float speed_sensitivity;     // 速度敏感性
            float terrain_sensitivity;   // 地形敏感性
            float traffic_sensitivity;   // 交通敏感性
            
            void adapt_to_context(const DrivingContext& context);
        } contextual_params_;
        
        float apply_contextual_adjustments(float base_prediction, 
                                          const DrivingContext& context);
    };
    
    // 预测性参数缓存
    class PredictiveParameterCache {
    public:
        struct CachedParameters {
            uint32_t target_timestamp;
            DrivingContext expected_context;
            LightweightMLPredictor::ModelParameters parameters;
            float confidence;
            bool is_used;
        };
        
        PredictiveParameterCache();
        
        void cache_parameters_for_future(uint32_t future_timestamp,
                                        const DrivingContext& context,
                                        const LightweightMLPredictor::ModelParameters& params,
                                        float confidence);
        
        std::optional<LightweightMLPredictor::ModelParameters> 
        get_best_cached_parameters(uint32_t current_time, 
                                  const DrivingContext& current_context);
        
        void cleanup_expired_cache(uint32_t current_time);
        
        float get_cache_hit_rate() const;
        
    private:
        static constexpr size_t CACHE_SIZE = 20;
        std::array<CachedParameters, CACHE_SIZE> cache_;
        size_t cache_index_;
        uint32_t cache_hits_;
        uint32_t cache_requests_;
        
        float calculate_context_similarity(const DrivingContext& ctx1, 
                                          const DrivingContext& ctx2) const;
    };
    
    // 自适应策略管理器
    class AdaptationStrategyManager {
    public:
        AdaptationStrategyManager();
        
        AdaptationStrategy decide_strategy(const SystemLoadState& load_state,
                                         const DrivingContext& context,
                                         float recent_accuracy);
        
        void update_strategy_effectiveness(const AdaptationStrategy& strategy,
                                         float accuracy_improvement);
        
    private:
        struct StrategyEffectiveness {
            AdaptationStrategy strategy;
            float effectiveness_score;
            uint32_t usage_count;
        };
        
        std::array<StrategyEffectiveness, 10> strategy_history_;
        size_t strategy_index_;
        
        float calculate_strategy_score(const AdaptationStrategy& strategy,
                                     const SystemLoadState& load_state,
                                     const DrivingContext& context) const;
    };
    
    // 通信调度器
    class CommunicationScheduler {
    public:
        CommunicationScheduler(std::shared_ptr<can::ICanInterface> can_interface);
        
        void schedule_parameter_request(const DrivingContext& predicted_context,
                                      uint32_t target_timestamp);
        
        void set_communication_strategy(AdaptationStrategy::CommunicationMode mode);
        
        bool has_pending_parameters() const;
        std::optional<LightweightMLPredictor::ModelParameters> get_latest_parameters();
        
    private:
        std::shared_ptr<can::ICanInterface> can_interface_;
        AdaptationStrategy::CommunicationMode current_mode_;
        
        struct PendingRequest {
            DrivingContext context;
            uint32_t target_timestamp;
            bool is_sent;
        };
        
        std::array<PendingRequest, 5> pending_requests_;
        size_t request_index_;
        
        void send_parameter_request_via_can(const DrivingContext& context);
        void process_incoming_parameters();
    };
    
    // 性能监控器
    class PerformanceMonitor {
    public:
        PerformanceMonitor();
        
        void record_prediction(float prediction, uint32_t inference_time_us);
        void record_accuracy(float accuracy);
        void record_cache_hit(bool hit);
        void record_adaptation();
        
        PerformanceStats get_statistics() const;
        
        float get_recent_accuracy() const;
        float get_system_efficiency_score() const;
        
    private:
        struct Statistics {
            uint32_t total_predictions;
            float accuracy_sum;
            uint32_t inference_time_sum;
            uint32_t cache_hits;
            uint32_t cache_requests;
            uint32_t adaptations;
        } stats_;
        
        std::array<float, 100> recent_accuracies_;
        size_t accuracy_index_;
    };
    
    // 后台任务管理
    void start_background_tasks();
    void stop_background_tasks();
    
    void adaptation_task();          // 自适应策略调整任务
    void caching_task();            // 预测性缓存任务
    void communication_task();      // 通信调度任务
    void monitoring_task();         // 性能监控任务
    
    // 成员变量
    PlatformInterface* platform_;
    std::shared_ptr<can::ICanInterface> can_interface_;
    
    // 核心组件
    std::unique_ptr<LocalAdaptivePredictor> local_predictor_;
    std::unique_ptr<PredictiveParameterCache> parameter_cache_;
    std::unique_ptr<AdaptationStrategyManager> strategy_manager_;
    std::unique_ptr<CommunicationScheduler> comm_scheduler_;
    std::unique_ptr<PerformanceMonitor> performance_monitor_;
    
    // 平台抽象接口
    std::unique_ptr<ThreadInterface> adaptation_thread_;
    std::unique_ptr<ThreadInterface> caching_thread_;
    std::unique_ptr<ThreadInterface> communication_thread_;
    std::unique_ptr<ThreadInterface> monitoring_thread_;
    std::unique_ptr<MutexInterface> data_mutex_;
    std::unique_ptr<TimeInterface> time_interface_;
    
    // 当前状态
    DrivingContext current_context_;
    SystemLoadState current_load_state_;
    AdaptationStrategy current_strategy_;
    
    // 运行状态
    bool is_initialized_;
    bool is_running_;
    
    // 预测历史 (用于在线学习)
    struct PredictionHistory {
        float prediction;
        float actual_load;
        uint32_t timestamp;
        DrivingContext context;
    };
    
    std::array<PredictionHistory, 200> prediction_history_;
    size_t history_index_;
};

} // namespace prediction
} // namespace vcu

#endif // ADAPTIVE_PREDICTION_SYSTEM_H
