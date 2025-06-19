#pragma once

#include <filesystem>

#include <tbai_core/Types.hpp>

constexpr const char *INIT_TIME_FILE = "/tmp/tbai_init_time_123";

namespace tbai {

void writeInitTime();
void writeInitTime(const long seconds, const long nanoseconds);
void writeInitTime(const scalar_t time);

template <typename TIMEPOINT>
inline scalar_t convertToScalar(const TIMEPOINT &time) {
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count();
    auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch() % std::chrono::seconds(1)).count();
    return static_cast<scalar_t>(seconds) + static_cast<scalar_t>(nanoseconds) * 1e-9;
}

scalar_t readInitTime();

std::string downloadFromHuggingFace(const std::string &repo_id, const std::string &filename);

}  // namespace tbai