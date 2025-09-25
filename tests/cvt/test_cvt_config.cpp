#include <gtest/gtest.h>
#include <fstream>
#include <cstdlib>
#include "vcu/cvt/cvt_config.h"

namespace vcu {
namespace cvt {
namespace test {

class CvtConfigTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a temporary config file for testing
        test_config_file_ = "/tmp/test_cvt_config.txt";
    }

    void TearDown() override {
        // Clean up test files
        std::remove(test_config_file_.c_str());
        
        // Clean up environment variables
        unsetenv("VCU_CVT_MANUFACTURER");
        unsetenv("VCU_CVT_MIN_RATIO");
        unsetenv("VCU_CVT_MAX_RATIO");
        unsetenv("VCU_CVT_DEBUG");
        unsetenv("VCU_CVT_LOG_PATH");
    }

    void create_test_config_file(const std::string& content) {
        std::ofstream file(test_config_file_);
        file << content;
        file.close();
    }

    std::string test_config_file_;
};

TEST_F(CvtConfigTest, DefaultConfigIsValid) {
    CvtConfig config = CvtConfigManager::get_default_config();
    
    EXPECT_TRUE(CvtConfigManager::validate_config(config));
    EXPECT_EQ(config.manufacturer, common::CvtManufacturer::HMCVT_VENDOR1);
    EXPECT_EQ(config.min_ratio, -0.9f);
    EXPECT_EQ(config.max_ratio, 2.0f);
    EXPECT_GT(config.ratio_tolerance, 0.0f);
    EXPECT_GT(config.adjustment_rate, 0.0f);
    EXPECT_GT(config.control_period_ms, 0);
    EXPECT_GT(config.status_period_ms, 0);
}

TEST_F(CvtConfigTest, LoadConfigFromValidFile) {
    std::string config_content = R"(
# Test CVT Configuration
manufacturer=HMCVT_VENDOR1
min_ratio=-0.8
max_ratio=1.8
ratio_tolerance=0.02
adjustment_rate=0.15
control_period_ms=20
status_period_ms=150
max_oil_temp=85.0
min_pressure=20.0
min_engine_rpm=1200
control_can_id=18FFF023
status_can_id=18FFF024
enable_four_wheel_drive=true
enable_differential_lock=false
enable_auto_clutch=true
enable_debug_logging=true
log_file_path=/tmp/cvt_debug.log
)";

    create_test_config_file(config_content);
    
    CvtConfig config = CvtConfigManager::load_from_file(test_config_file_);
    
    EXPECT_TRUE(CvtConfigManager::validate_config(config));
    EXPECT_EQ(config.manufacturer, common::CvtManufacturer::HMCVT_VENDOR1);
    EXPECT_FLOAT_EQ(config.min_ratio, -0.8f);
    EXPECT_FLOAT_EQ(config.max_ratio, 1.8f);
    EXPECT_FLOAT_EQ(config.ratio_tolerance, 0.02f);
    EXPECT_FLOAT_EQ(config.adjustment_rate, 0.15f);
    EXPECT_EQ(config.control_period_ms, 20);
    EXPECT_EQ(config.status_period_ms, 150);
    EXPECT_FLOAT_EQ(config.max_oil_temp, 85.0f);
    EXPECT_FLOAT_EQ(config.min_pressure, 20.0f);
    EXPECT_EQ(config.min_engine_rpm, 1200);
    EXPECT_EQ(config.control_can_id, 0x18FFF023u);
    EXPECT_EQ(config.status_can_id, 0x18FFF024u);
    EXPECT_TRUE(config.enable_four_wheel_drive);
    EXPECT_FALSE(config.enable_differential_lock);
    EXPECT_TRUE(config.enable_auto_clutch);
    EXPECT_TRUE(config.enable_debug_logging);
    EXPECT_EQ(config.log_file_path, "/tmp/cvt_debug.log");
}

TEST_F(CvtConfigTest, LoadConfigFromNonExistentFile) {
    CvtConfig config = CvtConfigManager::load_from_file("/non/existent/file.txt");
    
    // Should return default config when file doesn't exist
    CvtConfig default_config = CvtConfigManager::get_default_config();
    EXPECT_EQ(config.manufacturer, default_config.manufacturer);
    EXPECT_FLOAT_EQ(config.min_ratio, default_config.min_ratio);
    EXPECT_FLOAT_EQ(config.max_ratio, default_config.max_ratio);
}

TEST_F(CvtConfigTest, LoadConfigFromEnvironment) {
    // Set environment variables
    setenv("VCU_CVT_MANUFACTURER", "BOSCH", 1);
    setenv("VCU_CVT_MIN_RATIO", "-0.7", 1);
    setenv("VCU_CVT_MAX_RATIO", "1.9", 1);
    setenv("VCU_CVT_DEBUG", "true", 1);
    setenv("VCU_CVT_LOG_PATH", "/tmp/env_test.log", 1);
    
    CvtConfig config = CvtConfigManager::load_from_environment();
    
    EXPECT_TRUE(CvtConfigManager::validate_config(config));
    EXPECT_EQ(config.manufacturer, common::CvtManufacturer::BOSCH);
    EXPECT_FLOAT_EQ(config.min_ratio, -0.7f);
    EXPECT_FLOAT_EQ(config.max_ratio, 1.9f);
    EXPECT_TRUE(config.enable_debug_logging);
    EXPECT_EQ(config.log_file_path, "/tmp/env_test.log");
}

TEST_F(CvtConfigTest, ValidateConfigWithInvalidRatioRange) {
    CvtConfig config = CvtConfigManager::get_default_config();
    config.min_ratio = 1.0f;
    config.max_ratio = 0.5f; // Invalid: min > max
    
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
}

TEST_F(CvtConfigTest, ValidateConfigWithOutOfBoundsRatios) {
    CvtConfig config = CvtConfigManager::get_default_config();
    
    // Test min ratio out of bounds
    config.min_ratio = -3.0f;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
    
    // Reset and test max ratio out of bounds
    config = CvtConfigManager::get_default_config();
    config.max_ratio = 4.0f;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
}

TEST_F(CvtConfigTest, ValidateConfigWithInvalidTolerance) {
    CvtConfig config = CvtConfigManager::get_default_config();
    
    // Test zero tolerance
    config.ratio_tolerance = 0.0f;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
    
    // Test negative tolerance
    config.ratio_tolerance = -0.01f;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
    
    // Test too large tolerance
    config.ratio_tolerance = 0.2f;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
}

TEST_F(CvtConfigTest, ValidateConfigWithInvalidPeriods) {
    CvtConfig config = CvtConfigManager::get_default_config();
    
    // Test zero control period
    config.control_period_ms = 0;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
    
    // Test too large control period
    config = CvtConfigManager::get_default_config();
    config.control_period_ms = 200;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
    
    // Test zero status period
    config = CvtConfigManager::get_default_config();
    config.status_period_ms = 0;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
    
    // Test too large status period
    config = CvtConfigManager::get_default_config();
    config.status_period_ms = 2000;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
}

TEST_F(CvtConfigTest, ValidateConfigWithInvalidSafetyParams) {
    CvtConfig config = CvtConfigManager::get_default_config();
    
    // Test invalid oil temperature
    config.max_oil_temp = 200.0f;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
    
    // Test invalid pressure
    config = CvtConfigManager::get_default_config();
    config.min_pressure = 100.0f;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
    
    // Test invalid engine RPM
    config = CvtConfigManager::get_default_config();
    config.min_engine_rpm = 5000;
    EXPECT_FALSE(CvtConfigManager::validate_config(config));
}

TEST_F(CvtConfigTest, SaveConfigToFile) {
    CvtConfig config = CvtConfigManager::get_default_config();
    config.manufacturer = common::CvtManufacturer::BOSCH;
    config.enable_debug_logging = true;
    config.log_file_path = "/tmp/test_save.log";
    
    EXPECT_TRUE(CvtConfigManager::save_to_file(config, test_config_file_));
    
    // Verify file was created and can be read back
    CvtConfig loaded_config = CvtConfigManager::load_from_file(test_config_file_);
    
    EXPECT_EQ(loaded_config.manufacturer, common::CvtManufacturer::BOSCH);
    EXPECT_TRUE(loaded_config.enable_debug_logging);
    EXPECT_EQ(loaded_config.log_file_path, "/tmp/test_save.log");
}

TEST_F(CvtConfigTest, SaveInvalidConfigFails) {
    CvtConfig invalid_config;
    invalid_config.min_ratio = 2.0f;
    invalid_config.max_ratio = 1.0f; // Invalid range
    
    EXPECT_FALSE(CvtConfigManager::save_to_file(invalid_config, test_config_file_));
}

TEST_F(CvtConfigTest, ManufacturerStringParsing) {
    std::string config_content = R"(
manufacturer=ZF
)";

    create_test_config_file(config_content);
    
    CvtConfig config = CvtConfigManager::load_from_file(test_config_file_);
    
    EXPECT_EQ(config.manufacturer, common::CvtManufacturer::ZF);
}

TEST_F(CvtConfigTest, BooleanParsing) {
    std::string config_content = R"(
enable_four_wheel_drive=false
enable_differential_lock=true
enable_auto_clutch=false
enable_debug_logging=true
)";

    create_test_config_file(config_content);
    
    CvtConfig config = CvtConfigManager::load_from_file(test_config_file_);
    
    EXPECT_FALSE(config.enable_four_wheel_drive);
    EXPECT_TRUE(config.enable_differential_lock);
    EXPECT_FALSE(config.enable_auto_clutch);
    EXPECT_TRUE(config.enable_debug_logging);
}

TEST_F(CvtConfigTest, HexCanIdParsing) {
    std::string config_content = R"(
control_can_id=18FFF025
status_can_id=18FFF026
)";

    create_test_config_file(config_content);
    
    CvtConfig config = CvtConfigManager::load_from_file(test_config_file_);
    
    EXPECT_EQ(config.control_can_id, 0x18FFF025u);
    EXPECT_EQ(config.status_can_id, 0x18FFF026u);
}

} // namespace test
} // namespace cvt
} // namespace vcu
