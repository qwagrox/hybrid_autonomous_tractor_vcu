#include "../../include/utils/config_parser.hpp"
#include <stdexcept>

bool ConfigParser::loadFile(const std::string& file_path) {
    try {
        root_node_ = YAML::LoadFile(file_path);
        loaded_ = true;
        return true;
    } catch (const YAML::Exception& e) {
        // 在实际应用中，这里应该使用更健壮的日志记录
        fprintf(stderr, "Error loading YAML file %s: %s\n", file_path.c_str(), e.what());
        loaded_ = false;
        return false;
    }
}

YAML::Node ConfigParser::getNode() const {
    if (!loaded_) {
        throw std::runtime_error("Config file not loaded.");
    }
    return root_node_;
}

