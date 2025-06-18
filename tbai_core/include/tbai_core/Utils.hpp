#pragma once

#include <filesystem>

#include <tbai_core/Types.hpp>

constexpr const char *INIT_TIME_FILE = "/tmp/tbai_init_time_123";

namespace tbai {

void writeInitTime();
void writeInitTime(scalar_t initTime);

scalar_t readInitTime();

std::string downloadFromHuggingFace(const std::string &repo_id, const std::string &filename);

}  // namespace tbai