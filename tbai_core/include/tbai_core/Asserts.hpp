#pragma once

#ifdef TBAI_DISABLE_ASSERTS
#define TBAI_ASSERT(condition, ...)
#else

#include <fmt/format.h>
#include <fmt/ranges.h>

#define TBAI_ASSERT(condition, ...)                                                                               \
    do {                                                                                                          \
        if (!(condition)) {                                                                                       \
            std::cerr << "\n"                                                                                     \
                      << "Assertion failed: " << #condition << " in file " << __FILE__ << " at line " << __LINE__ \
                      << ": " << fmt::format(__VA_ARGS__) << "\n"                                                 \
                      << std::endl;                                                                               \
            std::abort();                                                                                         \
        }                                                                                                         \
    } while (0)

#endif