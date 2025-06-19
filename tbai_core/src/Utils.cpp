#include <filesystem>
#include <fstream>

#include <fmt/format.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Files.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/Utils.hpp>

namespace tbai {

static constexpr const char *INIT_TIME_FILE = "/tmp/tbai_init_time_123";
static constexpr const char *TBAI_HF_CACHE_DIR = "/tmp/tbai_hf_cache";

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void writeInitTime() {
    auto now = std::chrono::system_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() % std::chrono::seconds(1)).count();
    writeInitTime(seconds, nanoseconds);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void writeInitTime(const long seconds, const long nanoseconds) {
    writeInitTime(static_cast<scalar_t>(seconds) + static_cast<scalar_t>(nanoseconds) * 1e-9);
}
/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void writeInitTime(const scalar_t time) {
    // Remove the file if it exists
    if (std::filesystem::exists(INIT_TIME_FILE)) {
        std::filesystem::remove(INIT_TIME_FILE);
    }

    auto filelock = FileLock::lock(INIT_TIME_FILE, true);

    std::ofstream file(INIT_TIME_FILE);
    file << std::fixed << std::setprecision(20) << time;
    file.close();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
scalar_t readInitTime() {
    TBAI_THROW_UNLESS(std::filesystem::exists(INIT_TIME_FILE),
                      fmt::format("Initialization time file does not exist: {}", INIT_TIME_FILE));

    // Get lock of the file
    auto filelock = FileLock::lock(INIT_TIME_FILE);

    // Read the initialization time from the file
    std::ifstream file(INIT_TIME_FILE);
    scalar_t initTime;
    file >> initTime;
    file.close();

    return initTime;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::string downloadFromHuggingFace(const std::string &repo_id, const std::string &filename) {
    // Get the cache directory
    std::string cache_dir = getEnvAs<std::string>("TBAI_CACHE_DIR", true, TBAI_HF_CACHE_DIR);

    // Create the cache directory if it doesn't exist
    if (!std::filesystem::exists(cache_dir)) {
        std::filesystem::create_directories(cache_dir);
    }

    std::string command = fmt::format("huggingface-cli download {0} {1} --local-dir {2}", repo_id, filename, cache_dir);

    // Execute the command
    int result = system(command.c_str());
    TBAI_THROW_UNLESS(
        result == 0,
        fmt::format("Failed to download file from Hugging Face: {}\nMake sure huggingface-cli is installed.", command));

    std::string path = cache_dir + "/" + filename;
    return path;
}

}  // namespace tbai