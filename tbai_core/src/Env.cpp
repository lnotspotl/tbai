#include <tbai_core/Env.hpp>
#include <tbai_core/Throws.hpp>

namespace tbai {

void setEnv(const std::string &var, const std::string &value) {
    if (setenv(var.c_str(), value.c_str(), 1) != 0) {
        TBAI_THROW("Failed to set environment variable " + var);
    }
}

void unsetEnv(const std::string &var) {
    if (unsetenv(var.c_str()) != 0) {
        TBAI_THROW("Failed to unset environment variable " + var);
    }
}

}  // namespace tbai