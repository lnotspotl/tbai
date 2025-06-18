#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <tbai_core/Throws.hpp>

namespace tbai {

template <typename T>
T getEnvAs(const std::string &var, T defaultValue, bool allowDefault = true) {
    const char *value = std::getenv(var.c_str());
    if (value == nullptr) {
        if (allowDefault) {
            return defaultValue;
        }
        TBAI_THROW("Environment variable " + var + " is not set. Defaults are not allowed.");
    }
    return value;
}

template <typename T>
T getEnvAsChecked(const std::string &var, T defaultValue, const std::vector<T> &allowedValues,
                  bool allowDefault = true) {
    T value = getEnvAs<T>(var, defaultValue, allowDefault);
    if (std::find(allowedValues.begin(), allowedValues.end(), value) == allowedValues.end()) {
        TBAI_THROW("Environment variable " + var + " has invalid value: " + std::to_string(value));
    }
    return value;
}

void setEnv(const std::string &var, const std::string &value);

}  // namespace tbai