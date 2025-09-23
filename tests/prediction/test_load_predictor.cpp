#include <gtest/gtest.h>
#include "vcu/prediction/load_predictor.h"
#include <chrono>
#include <thread>

namespace vcu {
namespace prediction {
namespace test {

class LoadPredictorTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.history_window_size = 5;
        config_.prediction_horizon_ms = 1000;
        config_.terrain_weight = 0.3f;
        config_.speed_weight = 0.4f;
        config_.load_weight = 0.3f;
        
        predictor_ = std::make_unique<LoadPredictor>(config_);
    }

    void TearDown() override {
        predictor_.reset();
    }

    common::PerceptionData create_test_data(float speed, float load, common::TerrainType terrain) {
        common::PerceptionData data;
        data.vehicle_speed_mps = speed;
        data.engine_load_percent = load;
        data.engine_speed_rpm = 1500.0f;
        data.terrain_type = terrain;
        data.data_valid = true;
        return data;
    }

    LoadPredictionConfig config_;
    std::unique_ptr<LoadPredictor> predictor_;
};

TEST_F(LoadPredictorTest, InitialState) {
    EXPECT_FALSE(predictor_->has_sufficient_data());
    EXPECT_EQ(predictor_->get_history_size(), 0u);
    
    common::PredictionResult result;
    auto prediction_result = predictor_->predict_load(result);
    EXPECT_EQ(prediction_result, PredictionResult::ERROR_INIT);
}

TEST_F(LoadPredictorTest, SuccessfulInitialization) {
    auto result = predictor_->initialize();
    
    EXPECT_EQ(result, PredictionResult::SUCCESS);
    EXPECT_FALSE(predictor_->has_sufficient_data()); // Still no data
    EXPECT_EQ(predictor_->get_history_size(), 0u);
}

TEST_F(LoadPredictorTest, InvalidConfiguration) {
    LoadPredictionConfig invalid_config;
    invalid_config.history_window_size = 0; // Invalid
    invalid_config.terrain_weight = 0.5f;
    invalid_config.speed_weight = 0.5f;
    invalid_config.load_weight = 0.5f; // Weights sum > 1.0
    
    auto invalid_predictor = std::make_unique<LoadPredictor>(invalid_config);
    auto result = invalid_predictor->initialize();
    
    EXPECT_EQ(result, PredictionResult::ERROR_INVALID_INPUT);
}

TEST_F(LoadPredictorTest, DataUpdate) {
    predictor_->initialize();
    
    auto test_data = create_test_data(10.0f, 60.0f, common::TerrainType::SMOOTH);
    auto result = predictor_->update_data(test_data);
    
    EXPECT_EQ(result, PredictionResult::SUCCESS);
    EXPECT_EQ(predictor_->get_history_size(), 1u);
    EXPECT_FALSE(predictor_->has_sufficient_data()); // Need at least 3 points
}

TEST_F(LoadPredictorTest, InsufficientDataPrediction) {
    predictor_->initialize();
    
    // Add only 2 data points (need 3 for prediction)
    auto data1 = create_test_data(10.0f, 50.0f, common::TerrainType::SMOOTH);
    auto data2 = create_test_data(12.0f, 55.0f, common::TerrainType::SMOOTH);
    
    predictor_->update_data(data1);
    predictor_->update_data(data2);
    
    common::PredictionResult prediction;
    auto result = predictor_->predict_load(prediction);
    
    EXPECT_EQ(result, PredictionResult::ERROR_INSUFFICIENT_DATA);
}

TEST_F(LoadPredictorTest, SuccessfulPrediction) {
    predictor_->initialize();
    
    // Add sufficient data points
    std::vector<float> loads = {50.0f, 55.0f, 60.0f, 58.0f};
    for (size_t i = 0; i < loads.size(); ++i) {
        auto data = create_test_data(10.0f + i, loads[i], common::TerrainType::SMOOTH);
        predictor_->update_data(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Ensure different timestamps
    }
    
    EXPECT_TRUE(predictor_->has_sufficient_data());
    
    common::PredictionResult prediction;
    auto result = predictor_->predict_load(prediction);
    
    EXPECT_EQ(result, PredictionResult::SUCCESS);
    EXPECT_GT(prediction.predicted_load_percent, 0.0f);
    EXPECT_LE(prediction.predicted_load_percent, 100.0f);
}

TEST_F(LoadPredictorTest, TerrainInfluence) {
    predictor_->initialize();
    
    // Add base data on smooth terrain
    std::vector<float> base_loads = {50.0f, 52.0f, 54.0f};
    for (float load : base_loads) {
        auto data = create_test_data(10.0f, load, common::TerrainType::SMOOTH);
        predictor_->update_data(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    common::PredictionResult smooth_prediction;
    predictor_->predict_load(smooth_prediction);
    
    // Add data point on rough terrain
    auto rough_data = create_test_data(10.0f, 54.0f, common::TerrainType::ROUGH);
    predictor_->update_data(rough_data);
    
    common::PredictionResult rough_prediction;
    predictor_->predict_load(rough_prediction);
    
    // Rough terrain should predict higher load
    EXPECT_GT(rough_prediction.predicted_load_percent, smooth_prediction.predicted_load_percent);
}

TEST_F(LoadPredictorTest, SpeedTrendInfluence) {
    predictor_->initialize();
    
    // Add data with increasing speed trend
    std::vector<std::pair<float, float>> speed_load_pairs = {
        {8.0f, 50.0f}, {10.0f, 52.0f}, {12.0f, 54.0f}, {15.0f, 56.0f}
    };
    
    for (const auto& pair : speed_load_pairs) {
        auto data = create_test_data(pair.first, pair.second, common::TerrainType::SMOOTH);
        predictor_->update_data(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    common::PredictionResult prediction;
    auto result = predictor_->predict_load(prediction);
    
    EXPECT_EQ(result, PredictionResult::SUCCESS);
    // With increasing speed, load should be predicted higher than base average
    EXPECT_GT(prediction.predicted_load_percent, 53.0f); // Above simple average
}

TEST_F(LoadPredictorTest, HistoryWindowLimit) {
    predictor_->initialize();
    
    // Add more data points than window size
    for (int i = 0; i < 10; ++i) {
        auto data = create_test_data(10.0f, 50.0f + i, common::TerrainType::SMOOTH);
        predictor_->update_data(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    // Should only keep the configured window size
    EXPECT_EQ(predictor_->get_history_size(), config_.history_window_size);
}

TEST_F(LoadPredictorTest, ConfigurationUpdate) {
    predictor_->initialize();
    
    // Add some data
    for (int i = 0; i < 3; ++i) {
        auto data = create_test_data(10.0f, 50.0f, common::TerrainType::SMOOTH);
        predictor_->update_data(data);
    }
    
    EXPECT_EQ(predictor_->get_history_size(), 3u);
    
    // Update configuration with smaller window
    LoadPredictionConfig new_config = config_;
    new_config.history_window_size = 2;
    predictor_->set_config(new_config);
    
    // History should be trimmed
    EXPECT_EQ(predictor_->get_history_size(), 2u);
}

TEST_F(LoadPredictorTest, ClearHistory) {
    predictor_->initialize();
    
    // Add some data
    for (int i = 0; i < 3; ++i) {
        auto data = create_test_data(10.0f, 50.0f, common::TerrainType::SMOOTH);
        predictor_->update_data(data);
    }
    
    EXPECT_GT(predictor_->get_history_size(), 0u);
    
    predictor_->clear_history();
    
    EXPECT_EQ(predictor_->get_history_size(), 0u);
    EXPECT_FALSE(predictor_->has_sufficient_data());
}

TEST_F(LoadPredictorTest, InvalidDataUpdate) {
    predictor_->initialize();
    
    common::PerceptionData invalid_data;
    invalid_data.data_valid = false; // Invalid data
    
    auto result = predictor_->update_data(invalid_data);
    
    EXPECT_EQ(result, PredictionResult::ERROR_INVALID_INPUT);
    EXPECT_EQ(predictor_->get_history_size(), 0u);
}

TEST_F(LoadPredictorTest, Shutdown) {
    predictor_->initialize();
    
    // Add some data
    auto data = create_test_data(10.0f, 50.0f, common::TerrainType::SMOOTH);
    predictor_->update_data(data);
    
    EXPECT_GT(predictor_->get_history_size(), 0u);
    
    auto result = predictor_->shutdown();
    
    EXPECT_EQ(result, PredictionResult::SUCCESS);
    EXPECT_EQ(predictor_->get_history_size(), 0u);
}

TEST_F(LoadPredictorTest, PredictionBounds) {
    predictor_->initialize();
    
    // Add data with very high loads
    std::vector<float> high_loads = {95.0f, 98.0f, 99.0f};
    for (float load : high_loads) {
        auto data = create_test_data(5.0f, load, common::TerrainType::ROUGH);
        predictor_->update_data(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    common::PredictionResult prediction;
    predictor_->predict_load(prediction);
    
    // Prediction should be clamped to valid range
    EXPECT_GE(prediction.predicted_load_percent, 0.0f);
    EXPECT_LE(prediction.predicted_load_percent, 100.0f);
}

} // namespace test
} // namespace prediction
} // namespace vcu
