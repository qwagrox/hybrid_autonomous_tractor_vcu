#include "../../include/hardware/pwm_driver.hpp"
#include <iostream>
#include <fstream>
#include <string>

// 这是一个针对典型Linux sysfs PWM接口的实现。
// 实际硬件平台的路径可能会有所不同。
const std::string PWM_BASE_PATH = "/sys/class/pwm/pwmchip0/";

PWMDriver::PWMDriver(int channel)
    : channel_(channel), initialized_(false) {}

PWMDriver::~PWMDriver() {
    if (initialized_) {
        disable();
        unexportChannel();
    }
}

bool PWMDriver::initialize() {
    if (exportChannel()) {
        initialized_ = true;
        return true;
    }
    return false;
}

bool PWMDriver::setDutyCycle(double percentage) {
    if (!initialized_ || percentage < 0.0 || percentage > 100.0) {
        return false;
    }
    long period_ns = getPeriod();
    if (period_ns <= 0) return false;

    long duty_cycle_ns = static_cast<long>((period_ns * percentage) / 100.0);
    return writeValue(getChannelPath() + "duty_cycle", duty_cycle_ns);
}

bool PWMDriver::setFrequency(double freq_hz) {
    if (!initialized_ || freq_hz <= 0.0) {
        return false;
    }
    long period_ns = static_cast<long>(1.0e9 / freq_hz);
    return writeValue(getChannelPath() + "period", period_ns);
}

bool PWMDriver::enable() {
    if (!initialized_) return false;
    return writeValue(getChannelPath() + "enable", 1);
}

bool PWMDriver::disable() {
    if (!initialized_) return false;
    return writeValue(getChannelPath() + "enable", 0);
}

std::string PWMDriver::getChannelPath() const {
    return PWM_BASE_PATH + "pwm" + std::to_string(channel_) + "/";
}

bool PWMDriver::exportChannel() {
    return writeValue(PWM_BASE_PATH + "export", channel_);
}

bool PWMDriver::unexportChannel() {
    return writeValue(PWM_BASE_PATH + "unexport", channel_);
}

bool PWMDriver::writeValue(const std::string& path, long value) {
    try {
        std::ofstream ofs(path);
        if (!ofs.is_open()) {
            std::cerr << "Error: Unable to open PWM path: " << path << std::endl;
            return false;
        }
        ofs << value;
        ofs.close();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error writing to PWM path: " << e.what() << std::endl;
        return false;
    }
}

long PWMDriver::getPeriod() {
    try {
        std::ifstream ifs(getChannelPath() + "period");
        if (!ifs.is_open()) {
            return -1;
        }
        long period;
        ifs >> period;
        ifs.close();
        return period;
    } catch (const std::exception& e) {
        return -1;
    }
}

