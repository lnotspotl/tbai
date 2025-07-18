#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <tbai_core/Throws.hpp>

namespace tbai {

template <typename T>
T getEnvAs(const std::string &var, bool allowDefault = false, T defaultValue = T()) {
    const char *value = std::getenv(var.c_str());
    if (value == nullptr) {
        if (allowDefault) {
            return defaultValue;
        }
        TBAI_THROW("Environment variable {} is not set. Defaults are not allowed.", var);
    }
    return value;
}

template <>
bool getEnvAs(const std::string &var, bool allowDefault, bool defaultValue) {
    std::string value = getEnvAs<std::string>(var, allowDefault, defaultValue);
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    return (value == "true" || value == "1" || value == "yes" || value == "on");
}

template <typename T>
T getEnvAsChecked(const std::string &var, const std::vector<T> &allowedValues, bool allowDefault = false,
                  T defaultValue = T()) {
    T value = getEnvAs<T>(var, allowDefault, defaultValue);
    if (std::find(allowedValues.begin(), allowedValues.end(), value) == allowedValues.end()) {
        TBAI_THROW("Environment variable {} has invalid value: '{}'.\nExpected one of {}", var, value, allowedValues);
    }
    return value;
}

void setEnv(const std::string &var, const std::string &value);
void unsetEnv(const std::string &var);

}  // namespace tbai