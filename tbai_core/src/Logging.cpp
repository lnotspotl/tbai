#include "tbai_core/Logging.hpp"

#include <mutex>

#include "spdlog/spdlog.h"
#include <spdlog/async.h>

namespace tbai {

static std::shared_ptr<spdlog::details::thread_pool> globalThreadPool;

std::shared_ptr<spdlog::logger> getLogger(const std::string &name) {
    auto logLevel = tbai::getEnvAsChecked<std::string>(
        "TBAI_LOG_LEVEL", {"trace", "debug", "info", "warn", "error", "fatal"}, true, "info");
    auto logFolder = tbai::getEnvAs<std::string>("TBAI_LOG_FOLDER", true, "");
    auto logToConsole = tbai::getEnvAs<bool>("TBAI_LOG_TO_CONSOLE", true, true);
    auto useAsyncLogging = tbai::getEnvAs<bool>("TBAI_LOG_USE_ASYNC", true, false);

    std::vector<spdlog::sink_ptr> sinks;
    if (logToConsole) {
        sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    }

    if (!logFolder.empty()) {
        sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(logFolder + "/" + name + ".logs", true));
    }

    std::shared_ptr<spdlog::logger> logger;
    if (useAsyncLogging) {
        static std::mutex globalThreadPoolMutex;
        std::lock_guard<std::mutex> lock(globalThreadPoolMutex);

        if (!globalThreadPool) {
            globalThreadPool = std::make_shared<spdlog::details::thread_pool>(8192, 2);
        }

        logger = std::make_shared<spdlog::async_logger>(name, sinks.begin(), sinks.end(), globalThreadPool,
                                                        spdlog::async_overflow_policy::block);
        spdlog::register_logger(logger);
    } else {
        logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
    }
    logger->set_level(spdlog::level::from_str(logLevel));
    logger->set_pattern("[%H:%M:%S.%e] [%^%l%$] [%n] %v");
    return logger;
}

}  // namespace tbai
