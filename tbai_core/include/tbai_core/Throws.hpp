#pragma once

#include <iostream>

#define TBAI_THROW(message)                                                                                        \
    do {                                                                                                           \
        std::cerr << "\n"                                                                                          \
                  << "Exception thrown in file " << __FILE__ << " on line " << __LINE__ << ": " << message << "\n" \
                  << std::endl;                                                                                    \
        throw std::runtime_error(message);                                                                         \
    } while (0);

#define TBAI_THROW_IF(condition, message) \
    if (condition) {                      \
        TBAI_THROW(message);              \
    }

#define TBAI_THROW_UNLESS(condition, message) \
    if (!(condition)) {                       \
        TBAI_THROW(message);                  \
    }

#define TBAI_THROW_UNIMPLEMENTED() TBAI_THROW("Unimplemented function called");