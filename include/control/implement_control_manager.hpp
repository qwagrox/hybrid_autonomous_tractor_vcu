// include/control/implement_control_manager.hpp
#pragma once
#include "vcu_core_types.hpp"
#include "i_implement_controller.hpp"
#include <memory>
#include <vector>
#include <map>

namespace VCUCore {

/**
 * @class ImplementControlManager
 * @brief 农具控制管理器，负责管理多个农具控制器
 */
class ImplementControlManager {
private:
    std::vector<std::unique_ptr<IImplementController>> controllers_;
    std::map<std::string, size_t> controllerMap_;
    bool isInitialized_;

public:
    ImplementControlManager();
    ~ImplementControlManager() = default;

    bool initialize();
    void shutdown();
    
    bool addController(std::unique_ptr<IImplementController> controller);
    bool removeController(const std::string& type);
    
    IImplementController* getController(const std::string& type);
    std::vector<std::string> getAvailableTypes() const;
    
    bool startAll();
    bool stopAll();
    
    bool setWorkParameter(const std::string& type, const std::string& key, double value);
    std::vector<ImplementStatus> getAllStatus() const;
    std::vector<DiagnosticReport> runAllDiagnostics();
    
    void updateAll(double dt);
};

} // namespace VCUCore
