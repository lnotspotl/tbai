#include "tbai_core/Logging.hpp"

namespace tbai {

std::shared_ptr<spdlog::logger> getLogger(const std::string &name) {
    auto logLevel = tbai::getEnvAs<std::string>("TBAI_LOG_LEVEL", true, "info");
    auto logFolder = tbai::getEnvAs<std::string>("TBAI_LOG_FOLDER", true, "");
    auto logToConsole = tbai::getEnvAs<bool>("TBAI_LOG_TO_CONSOLE", true, true);

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());

    if (!logFolder.empty()) {
        sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(logFolder + "/" + name + ".logs", true));
    }

    auto logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
    logger->set_level(spdlog::level::from_str(logLevel));
    logger->set_pattern("[%H:%M:%S.%e] [%^%l%$] [%s:%#] %v");
    return logger;
}

}  // namespace tbai
