#include "tbai_core/Logging.hpp"

#include <filesystem>
#include <fstream>
#include <mutex>

#include "spdlog/spdlog.h"
#include <spdlog/async.h>
#include <tbai_core/Files.hpp>

namespace tbai {

static std::shared_ptr<spdlog::details::thread_pool> globalThreadPool;

std::shared_ptr<spdlog::logger> getLogger(const std::string &name) {
    static std::mutex getLoggerMutex;
    std::lock_guard<std::mutex> lock(getLoggerMutex);

    auto logLevel = tbai::getEnvAsChecked<std::string>(
        "TBAI_LOG_LEVEL", {"trace", "debug", "info", "warn", "error", "fatal"}, true, "info");
    auto logFolder = tbai::getEnvAs<std::string>("TBAI_LOG_FOLDER", true, "");
    auto logToConsole = tbai::getEnvAs<bool>("TBAI_LOG_TO_CONSOLE", true, true);
    auto useAsyncLogging = tbai::getEnvAs<bool>("TBAI_LOG_USE_ASYNC", true, false);
    auto fileLogLevel = tbai::getEnvAsChecked<std::string>(
        "TBAI_LOG_FILE_LEVEL", {"trace", "debug", "info", "warn", "error", "fatal"}, true, "debug");

    std::vector<spdlog::sink_ptr> sinks;
    if (logToConsole) {
        auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        consoleSink->set_pattern("[%H:%M:%S.%e] [%^%l%$] [%n] %v");
        consoleSink->set_level(spdlog::level::from_str(logLevel));
        sinks.push_back(consoleSink);
    }

    if (!logFolder.empty()) {
        static bool logFolderScript = false;
        if (!logFolderScript) {
            std::filesystem::create_directories(logFolder);
            auto fileLock = tbai::FileLock::lock(logFolder + "/combine_logs.sh", true);
            std::ofstream combineFile(logFolder + "/combine_logs.sh");
            combineFile << "sort -t']' -k1,1 ./*.logs > combined.txt" << std::endl;
            combineFile.close();
            logFolderScript = true;
        }

        auto logFile = logFolder + "/" + name + ".logs";
        if (logFolderScript) {
            auto fileSink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logFile, true);
            fileSink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v");
            fileSink->set_level(spdlog::level::from_str(fileLogLevel));
            sinks.push_back(fileSink);
        }
    }

    std::shared_ptr<spdlog::logger> logger;
    if (useAsyncLogging) {
        if (!globalThreadPool) {
            globalThreadPool = std::make_shared<spdlog::details::thread_pool>(8192, 2);
        }

        logger = std::make_shared<spdlog::async_logger>(name, sinks.begin(), sinks.end(), globalThreadPool,
                                                        spdlog::async_overflow_policy::block);
        spdlog::register_logger(logger);
    } else {
        logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
    }
    logger->set_level(std::min(spdlog::level::from_str(logLevel),
                               logFolder.empty() ? spdlog::level::off : spdlog::level::from_str(fileLogLevel)));
    return logger;
}

}  // namespace tbai
