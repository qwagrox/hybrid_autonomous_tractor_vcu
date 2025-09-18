// environment_model.hpp
#ifndef ENVIRONMENT_MODEL_HPP
#define ENVIRONMENT_MODEL_HPP

class EnvironmentModel {
public:
    EnvironmentModel() : slope_angle_(0.0), soil_resistance_(1.0) {}

    void update(double time) {
        // 模拟地形变化
        slope_angle_ = 5.0 * sin(time / 30.0); // 坡度周期性变化
    }

    double getSlopeAngle() const { return slope_angle_; }
    double getSoilResistance() const { return soil_resistance_; }

private:
    double slope_angle_; // degrees
    double soil_resistance_; // factor
};

#endif // ENVIRONMENT_MODEL_HPP

