#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP

#include <string>
#include <yaml-cpp/yaml.h>

/**
 * @class ConfigParser
 * @brief 一个简单的YAML配置文件解析器包装类
 *
 * 提供了一个静态方法来加载YAML文件，并提供了一个简单的接口来获取节点。
 */
class ConfigParser {
public:
    /**
     * @brief 加载一个YAML文件
     * @param file_path 文件的完整路径
     * @return 如果加载成功，返回true
     */
    bool loadFile(const std::string& file_path);

    /**
     * @brief 获取YAML文件的根节点
     * @return YAML::Node 根节点
     */
    YAML::Node getNode() const;

private:
    YAML::Node root_node_;
    bool loaded_ = false;
};

#endif // CONFIG_PARSER_HPP

