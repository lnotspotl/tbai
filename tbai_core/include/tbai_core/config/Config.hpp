#pragma once

#include <filesystem>

#include <tbai_core/Env.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/Types.hpp>
#include <yaml-cpp/yaml.h>

namespace tbai {

namespace {

constexpr char CONFIG_PATH_DELIM = '/';

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::vector<std::string> splitConfigPath(const std::string &s, char delim) {
    // Split config path: a.b.c -> {'a', 'b', 'c'} if delim_ == '.'
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        result.push_back(item);
    }
    return result;
}

}  // namespace

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
template <typename T>
T parseNode(const YAML::Node &node) {
    return node.as<T>();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
template <>
inline vector_t parseNode(const YAML::Node &node) {
    const size_t len = node.size();
    vector_t output(len);
    for (size_t i = 0; i < len; ++i) {
        output(i) = node[i].as<scalar_t>();
    }
    return output;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
template <>
inline matrix_t parseNode(const YAML::Node &node) {
    const size_t rows = node.size();
    const size_t cols = node[0].size();
    matrix_t output(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            output(i, j) = node[i][j].as<scalar_t>();
        }
    }
    return output;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
template <typename T>
T fromConfig(const std::string &path, const std::string &configPath) {
    // Make sure the config file exists
    TBAI_THROW_UNLESS(std::filesystem::exists(configPath), "Config file does not exist: " + configPath);

    // Load config
    YAML::Node config = YAML::LoadFile(configPath);

    // Traverse the config file
    YAML::Node component(config);
    for (auto &k : splitConfigPath(path, CONFIG_PATH_DELIM)) {
        if (!component[k]) {
            TBAI_THROW("Key '" + path + "' does not exist in config file.");
        }
        component = component[k];
    }

    // Parse value and return
    T value = parseNode<T>(component);
    return value;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
template <typename T>
T fromGlobalConfig(const std::string &path) {
    const std::string configPath = getEnvAs<std::string>("TBAI_GLOBAL_CONFIG_PATH", "");
    TBAI_THROW_UNLESS(!configPath.empty(), "TBAI_GLOBAL_CONFIG_PATH is not set");
    return fromConfig<T>(path, configPath);
}

}  // namespace tbai