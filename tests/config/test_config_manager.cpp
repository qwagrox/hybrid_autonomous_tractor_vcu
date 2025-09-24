// Copyright 2025 Manus AI

#include <gtest/gtest.h>
#include <fstream>
#include <cstdio>
#include "vcu/config/json_config_manager.h"

namespace vcu {
namespace config {

class ConfigManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_config_path_ = "/tmp/test_config.json";
    }

    void TearDown() override {
        std::remove(test_config_path_.c_str());
    }

    void create_test_config(const std::string& content) {
        std::ofstream config_file(test_config_path_);
        config_file << content;
        config_file.close();
    }

    std::string test_config_path_;
};

TEST_F(ConfigManagerTest, DefaultConfiguration) {
    JsonConfigManager config_manager;
    
    // Should use default config when no file is loaded
    auto config = config_manager.get_vcu_config();
    EXPECT_EQ(config.can_bus_name, "can0");
    EXPECT_EQ(config.data_timeout_ms, 1000);
}

TEST_F(ConfigManagerTest, LoadValidConfiguration) {
    create_test_config(R"({
        "can_bus_name": "can1",
        "data_timeout_ms": 2000
    })");

    JsonConfigManager config_manager;
    EXPECT_TRUE(config_manager.load_config(test_config_path_));
    
    auto config = config_manager.get_vcu_config();
    EXPECT_EQ(config.can_bus_name, "can1");
    EXPECT_EQ(config.data_timeout_ms, 2000);
}

TEST_F(ConfigManagerTest, LoadPartialConfiguration) {
    create_test_config(R"({
        "can_bus_name": "can2"
    })");

    JsonConfigManager config_manager;
    EXPECT_TRUE(config_manager.load_config(test_config_path_));
    
    auto config = config_manager.get_vcu_config();
    EXPECT_EQ(config.can_bus_name, "can2");
    EXPECT_EQ(config.data_timeout_ms, 1000); // Should keep default
}

TEST_F(ConfigManagerTest, LoadNonexistentFile) {
    JsonConfigManager config_manager;
    EXPECT_TRUE(config_manager.load_config("/nonexistent/path.json"));
    
    // Should use default configuration
    auto config = config_manager.get_vcu_config();
    EXPECT_EQ(config.can_bus_name, "can0");
    EXPECT_EQ(config.data_timeout_ms, 1000);
}

TEST_F(ConfigManagerTest, LoadInvalidJson) {
    create_test_config("invalid json content");

    JsonConfigManager config_manager;
    EXPECT_TRUE(config_manager.load_config(test_config_path_));
    
    // Should fallback to default configuration
    auto config = config_manager.get_vcu_config();
    EXPECT_EQ(config.can_bus_name, "can0");
    EXPECT_EQ(config.data_timeout_ms, 1000);
}

} // namespace config
} // namespace vcu
