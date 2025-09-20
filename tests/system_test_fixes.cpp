// tests/system_test.cpp - Lambda函数修复补丁
// 这个文件展示了如何修复system_test.cpp中的lambda函数返回类型问题

// 问题1: testSystemIntegration中的lambda函数没有返回值
// 原代码可能是：
/*
void SystemTester::testSystemIntegration() {
    runTest("Full System Loop", [this]() {
        // ... 测试代码 ...
        // 缺少return语句
    });
}
*/

// 修复方案：确保lambda函数返回bool值
void SystemTester::testSystemIntegration() {
    runTest("Full System Loop", [this]() -> bool {
        try {
            // 模拟感知数据
            PerceptionData perception;
            perception.vehicleState.position = Vector3d(0, 0, 0);
            perception.vehicleState.velocity = Vector3d(5, 0, 0);
            perception.vehicleState.heading = 0.0;
            perception.vehicleState.drawbarPull = 5000.0;
            perception.terrainSlope = 0.05;
            perception.rollingResistance = 0.02;
            perception.aerodynamicDrag = 0.3;
            perception.stabilityIndex = 0.8;
            perception.tractionEfficiency = 0.85;
            perception.timestamp = getCurrentTimestamp();

            // 传感器融合
            auto fusedState = sensorFusion_->fuseData(getSensorData());

            // 预测分析
            auto prediction = predictiveAnalytics_->predict(perception);
            prediction.predictedState = fusedState;
            prediction.confidence = 0.9;
            prediction.timestamp = getCurrentTimestamp();

            // 生成控制命令
            ControlCommands commands;
            commands.engineTorqueRequest = 1200.0;  // 使用double而不是float
            commands.motorTorqueRequest = 600.0;    // 使用double而不是float
            commands.cvtRatioRequest = cvtController_->calculateOptimalRatio(perception, prediction);
            commands.timestamp = getCurrentTimestamp();

            // 扭矩仲裁
            auto torqueSplit = torqueArbiter_->decideDistribution(perception, prediction);

            // 检查系统健康
            auto health = healthMonitor_->getSystemHealth();

            // 返回测试结果
            return health.isHealthy && torqueSplit.totalTorque > 0;
            
        } catch (const std::exception& e) {
            return false;
        }
    });
}

// 问题2: testHealthMonitoring中的lambda函数没有返回值
// 修复方案：
void SystemTester::testHealthMonitoring() {
    runTest("System Health Check", [this]() -> bool {
        try {
            auto health = healthMonitor_->getSystemHealth();
            
            // 检查健康度范围
            bool healthInRange = (health.overallHealth >= 0.0 && health.overallHealth <= 1.0);
            
            // 检查基本状态
            bool basicChecks = (health.batteryLevel >= 0.0 && health.batteryLevel <= 1.0) &&
                              (health.engineLoad >= 0.0 && health.engineLoad <= 1.0);
            
            return healthInRange && basicChecks;
            
        } catch (const std::exception& e) {
            return false;
        }
    });
}

// 辅助函数实现
uint64_t SystemTester::getCurrentTimestamp() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

SensorData SystemTester::getSensorData() const {
    SensorData data;
    data.gnssPosition = Vector3d(0, 0, 0);
    data.gnssVelocity = Vector3d(5, 0, 0);
    data.imuAcceleration = Vector3d(0, 0, 0);
    data.imuAngularRate = Vector3d(0, 0, 0);
    data.wheelSpeed = 5.0;
    data.wheelSpeeds = {5.0, 5.0, 5.0, 5.0}; // 四个车轮的速度
    data.engineRPM = 1800.0;
    data.engineTorque = 1000.0;
    data.fuelLevel = 0.8;
    data.batteryVoltage = 24.0;
    data.hydraulicPressure = 200.0;
    data.oilTemperature = 80.0;
    data.coolantTemperature = 85.0;
    data.timestamp = getCurrentTimestamp();
    return data;
}

// 如果需要在system_test.cpp中添加这些修复，请按照以下模式：

/*
在system_test.cpp中需要修改的地方：

1. 确保所有lambda函数都有明确的返回类型声明：
   [this]() -> bool { ... }

2. 确保所有lambda函数都有return语句：
   return some_boolean_expression;

3. 使用double字面量而不是float：
   1200.0 而不是 1200.0f

4. 确保所有异常都被捕获并返回适当的bool值：
   try { ... return true; } catch (...) { return false; }

5. 确保使用正确的方法名：
   healthMonitor_->getSystemHealth() 而不是其他变体
*/
