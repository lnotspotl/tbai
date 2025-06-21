#pragma once

#include <iostream>

#include <fmt/format.h>
#include <fmt/ranges.h>

#define TBAI_THROW(...)                                                                          \
    do {                                                                                         \
        std::string message = fmt::format(__VA_ARGS__);                                          \
        std::cerr << "\033[31m\n"                                                                \
                  << "Exception thrown in file " << __FILE__ << " on line " << __LINE__ << ":\n" \
                  << message << "\n"                                                             \
                  << "\033[0m" << std::endl;                                                     \
        throw std::runtime_error(message);                                                       \
    } while (0);

#define TBAI_THROW_IF(condition, ...) \
    if (condition) {                  \
        TBAI_THROW(__VA_ARGS__);      \
    }

#define TBAI_THROW_UNLESS(condition, ...) \
    if (!(condition)) {                   \
        TBAI_THROW(__VA_ARGS__);          \
    }
