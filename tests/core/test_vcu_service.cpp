#include <gtest/gtest.h>
#include "vcu/core/vcu_service.h"

namespace vcu {
namespace core {
namespace test {

class VcuServiceTest : public ::testing::Test {
protected:
    void SetUp() override {
        service_ = std::make_unique<VcuService>();
    }

    void TearDown() override {
        if (service_) {
            service_->shutdown();
        }
    }

    std::unique_ptr<VcuService> service_;
};

TEST_F(VcuServiceTest, InitialState) {
    EXPECT_EQ(service_->get_state(), VcuState::OFF);
}

TEST_F(VcuServiceTest, InitializeWithInvalidConfig) {
    // 使用不存在的配置文件
    EXPECT_FALSE(service_->initialize("/nonexistent/config.json"));
    EXPECT_EQ(service_->get_state(), VcuState::FAULT);
}

TEST_F(VcuServiceTest, ShutdownFromOffState) {
    // 从OFF状态关闭应该是安全的
    service_->shutdown();
    EXPECT_EQ(service_->get_state(), VcuState::OFF);
}

} // namespace test
} // namespace core
} // namespace vcu
