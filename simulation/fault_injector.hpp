// fault_injector.hpp
#ifndef FAULT_INJECTOR_HPP
#define FAULT_INJECTOR_HPP

#include "models/powertrain_model.hpp"
#include <string>
#include <vector>

enum class FaultType {
    SENSOR_FAILURE,
    ACTUATOR_STUCK,
    COMMUNICATION_LOST
};

struct FaultCondition {
    FaultType type;
    std::string target;
    double value;
    double start_time;
    double duration;
};

class FaultInjector {
public:
    void addFault(const FaultCondition& fault) {
        faults_.push_back(fault);
    }

    void apply(PowertrainModel& model, double time) {
        for (const auto& fault : faults_) {
            if (time >= fault.start_time && time < fault.start_time + fault.duration) {
                if (fault.type == FaultType::SENSOR_FAILURE && fault.target == "engine_speed") {
                    // 模拟传感器读数固定
                    // 实际应修改模型或VCU的输入
                }
            }
        }
    }

private:
    std::vector<FaultCondition> faults_;
};

#endif // FAULT_INJECTOR_HPP

