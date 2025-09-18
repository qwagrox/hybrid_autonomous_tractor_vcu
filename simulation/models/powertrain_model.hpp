// powertrain_model.hpp
#ifndef POWERTRAIN_MODEL_HPP
#define POWERTRAIN_MODEL_HPP

#include <iostream>
#include <cmath>

// 发动机模型
class EngineModel {
public:
    EngineModel() : speed_(0.0), torque_(0.0), throttle_position_(0.0) {}

    void update(double throttle, double load_torque, double dt) {
        // 简化的发动机动态模型
        double requested_torque = throttle * max_torque_;
        double net_torque = requested_torque - load_torque - speed_ * friction_;
        double acceleration = net_torque / inertia_;
        speed_ += acceleration * dt;

        if (speed_ < min_speed_) speed_ = min_speed_;
        if (speed_ > max_speed_) speed_ = max_speed_;

        torque_ = requested_torque;
        throttle_position_ = throttle;
    }

    double getSpeed() const { return speed_; }
    double getTorque() const { return torque_; }

private:
    double speed_; // RPM
    double torque_; // Nm
    double throttle_position_; // 0-1

    const double max_torque_ = 400.0; // Nm
    const double inertia_ = 10.0; // kg*m^2
    const double friction_ = 0.1; // Nms/rad
    const double max_speed_ = 2200.0; // RPM
    const double min_speed_ = 800.0; // RPM
};

// 电机模型
class MotorModel {
public:
    MotorModel() : speed_(0.0), torque_(0.0), command_(0.0) {}

    void update(double command, double load_torque, double dt) {
        command_ = command;
        double requested_torque = command * max_torque_;
        // 简单的电机模型，实际应更复杂
        torque_ = requested_torque - load_torque;
        speed_ += (torque_ / inertia_) * dt;
    }

    double getSpeed() const { return speed_; }
    double getTorque() const { return torque_; }
    double getPower() const { return torque_ * speed_ * (2 * M_PI / 60) / 1000; } // kW

private:
    double speed_;
    double torque_;
    double command_;

    const double max_torque_ = 200.0; // Nm
    const double inertia_ = 5.0;
};

// 电池模型
class BatteryModel {
public:
    BatteryModel() : soc_(100.0), voltage_(400.0) {}

    void update(double power_draw_kw, double dt) {
        double current = (power_draw_kw * 1000) / voltage_;
        double charge_used = current * dt / 3600; // Amp-hours
        double soc_change = (charge_used / capacity_ah_) * 100.0;
        soc_ -= soc_change;

        if (soc_ > 100.0) soc_ = 100.0;
        if (soc_ < 0.0) soc_ = 0.0;
    }

    double getSOC() const { return soc_; }
    double getVoltage() const { return voltage_; }

private:
    double soc_; // State of Charge (%)
    double voltage_; // V
    const double capacity_ah_ = 100.0; // Amp-hours
};

// CVT模型
class CvtModel {
public:
    CvtModel() : ratio_(2.0) {}

    void update(double command) {
        // 模拟CVT传动比变化
        ratio_ = command;
        if (ratio_ < min_ratio_) ratio_ = min_ratio_;
        if (ratio_ > max_ratio_) ratio_ = max_ratio_;
    }

    double getRatio() const { return ratio_; }

private:
    double ratio_;
    const double min_ratio_ = 0.5;
    const double max_ratio_ = 2.5;
};

// 完整的动力总成模型
class PowertrainModel {
public:
    void update(double engine_throttle, double motor_command, double cvt_command, double load_torque, double dt) {
        engine_.update(engine_throttle, load_torque * 0.5, dt); // 假设负载平分
        motor_.update(motor_command, load_torque * 0.5, dt);
        cvt_.update(cvt_command);

        double motor_power_kw = motor_.getPower();
        battery_.update(motor_power_kw, dt);
    }

    EngineModel& getEngine() { return engine_; }
    MotorModel& getMotor() { return motor_; }
    BatteryModel& getBattery() { return battery_; }
    CvtModel& getCvt() { return cvt_; }

private:
    EngineModel engine_;
    MotorModel motor_;
    BatteryModel battery_;
    CvtModel cvt_;
};

#endif // POWERTRAIN_MODEL_HPP

