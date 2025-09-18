#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "../../include/control/implement_control_manager.hpp"
#include "../../include/control/implement_drivers/plow_controller.hpp"
#include "../../include/integration/isobus_adapter.hpp"

// 使用Google Mock创建一个模拟的ISOBUSAdapter
class MockISOBUSAdapter : public ISOBUSAdapter {
public:
    MOCK_METHOD(bool, sendValueCommand, (uint8_t, uint32_t, double), (override));
    MOCK_METHOD(bool, sendEnableCommand, (uint8_t, uint32_t, bool), (override));
};

// 测试ImplementControlManager
TEST(ImplementControlManagerTest, SelectionAndCommandDispatch) {
    auto mock_adapter = std::make_shared<MockISOBUSAdapter>();
    auto manager = std::make_unique<ImplementControlManager>(mock_adapter);
    auto plow_driver = std::make_shared<PlowController>(mock_adapter);

    manager->registerDriver(plow_driver);
    EXPECT_TRUE(manager->selectImplement("Plow"));

    // 期望在设置参数时，sendValueCommand被调用一次
    EXPECT_CALL(*mock_adapter, sendValueCommand(_, _, _)).Times(1);
    manager->setWorkParameter("depth", 0.2);
}

// 测试PlowController
TEST(PlowControllerTest, ParameterSetting) {
    auto mock_adapter = std::make_shared<MockISOBUSAdapter>();
    auto plow_controller = std::make_unique<PlowController>(mock_adapter);

    ImplementConfig config;
    config.type = "Plow";
    plow_controller->initialize(config);
    plow_controller->start();

    // 期望调用sendValueCommand，PGN为0xEF00，值为0.3
    EXPECT_CALL(*mock_adapter, sendValueCommand(testing::_, 0xEF00, 0.3)).Times(1);
    plow_controller->setWorkParameter("depth", 0.3);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

