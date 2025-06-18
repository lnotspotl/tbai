#pragma once

#include <iostream>

#include <fmt/format.h>
#include <fmt/ranges.h>

#define TBAI_THROW(...)                                                                                            \
    do {                                                                                                           \
        std::string message = fmt::format(__VA_ARGS__);                                                            \
        std::cerr << "\n"                                                                                          \
                  << "Exception thrown in file " << __FILE__ << " on line " << __LINE__ << ": " << message << "\n" \
                  << std::endl;                                                                                    \
        throw std::runtime_error(message);                                                                         \
    } while (0);

#define TBAI_THROW_IF(condition, ...) \
    if (condition) {                  \
        TBAI_THROW(__VA_ARGS__);      \
    }

#define TBAI_THROW_UNLESS(condition, ...) \
    if (!(condition)) {                   \
        TBAI_THROW(__VA_ARGS__);          \
    }
