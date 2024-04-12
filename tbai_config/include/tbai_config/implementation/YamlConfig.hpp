#ifndef TBAI_CONFIG_INCLUDE_TBAI_CONFIG_IMPLEMENTATION_YAMLCONFIG_HPP_
#define TBAI_CONFIG_INCLUDE_TBAI_CONFIG_IMPLEMENTATION_YAMLCONFIG_HPP_

#include <iostream>
#include <string>
namespace tbai {

namespace config {

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
template <typename T>
T YamlConfig::traverse(const YAML::Node &node, const std::string &key) const {
    YAML::Node component(node);
    for (auto &k : split(key)) {
        component = component[k];
    }
    return parseNode<T>(component);
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
template <typename T>
inline T YamlConfig::parseNode(const YAML::Node &node) const {
    return node.as<T>();
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
template <typename T>
T YamlConfig::get(const std::string &path) const {
    YAML::Node config = YAML::LoadFile(configPath_);
    return traverse<T>(config, path);
}

}  // namespace config
}  // namespace tbai

#endif  // TBAI_CONFIG_INCLUDE_TBAI_CONFIG_IMPLEMENTATION_YAMLCONFIG_HPP_
