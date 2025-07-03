#pragma once

#include <chrono>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <tbai_core/Env.hpp>

#define TBAI_LOG_TRACE(logger, ...)                                                                                  \
    do {                                                                                                             \
        if (logger->should_log(spdlog::level::trace)) {                                                              \
            logger->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::trace, __VA_ARGS__); \
        }                                                                                                            \
    } while (false)
#define TBAI_LOG_DEBUG(logger, ...)                                                                                  \
    do {                                                                                                             \
        if (logger->should_log(spdlog::level::debug)) {                                                              \
            logger->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::debug, __VA_ARGS__); \
        }                                                                                                            \
    } while (false)
#define TBAI_LOG_INFO(logger, ...)                                                                                  \
    do {                                                                                                            \
        if (logger->should_log(spdlog::level::info)) {                                                              \
            logger->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::info, __VA_ARGS__); \
        }                                                                                                           \
    } while (false)
#define TBAI_LOG_WARN(logger, ...)                                                                                  \
    do {                                                                                                            \
        if (logger->should_log(spdlog::level::warn)) {                                                              \
            logger->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::warn, __VA_ARGS__); \
        }                                                                                                           \
    } while (false)
#define TBAI_LOG_ERROR(logger, ...)                                                                                \
    do {                                                                                                           \
        if (logger->should_log(spdlog::level::err)) {                                                              \
            logger->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::err, __VA_ARGS__); \
        }                                                                                                          \
    } while (false)
#define TBAI_LOG_FATAL(logger, ...)                                                                       \
    do {                                                                                                  \
        if (logger->should_log(spdlog::level::critical)) {                                                \
            logger->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::critical, \
                        __VA_ARGS__);                                                                     \
        }                                                                                                 \
    } while (false)

#define TBAI_GLOBAL_LOG_TRACE(...) SPDLOG_TRACE(__VA_ARGS__)
#define TBAI_GLOBAL_LOG_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define TBAI_GLOBAL_LOG_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define TBAI_GLOBAL_LOG_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define TBAI_GLOBAL_LOG_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define TBAI_GLOBAL_LOG_FATAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

#define __TBAI_LOG_THROTTLE_CHECK(now, last, period) (last + period <= now || now < last)

#define TBAI_GLOBAL_LOG_DEBUG_THROTTLE(period, ...)                                                     \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_GLOBAL_LOG_DEBUG(__VA_ARGS__);                                                         \
        }                                                                                               \
    } while (false)

#define TBAI_GLOBAL_LOG_INFO_THROTTLE(period, ...)                                                      \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_GLOBAL_LOG_INFO(__VA_ARGS__);                                                          \
        }                                                                                               \
    } while (false)

#define TBAI_GLOBAL_LOG_WARN_THROTTLE(period, ...)                                                      \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_GLOBAL_LOG_WARN(__VA_ARGS__);                                                          \
        }                                                                                               \
    } while (false)

#define TBAI_GLOBAL_LOG_ERROR_THROTTLE(period, ...)                                                     \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_GLOBAL_LOG_ERROR(__VA_ARGS__);                                                         \
        }                                                                                               \
    } while (false)

#define TBAI_GLOBAL_LOG_FATAL_THROTTLE(period, ...)                                                     \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_GLOBAL_LOG_FATAL(__VA_ARGS__);                                                         \
        }                                                                                               \
    } while (false)

#define TBAI_LOG_TRACE_THROTTLE(logger, period, ...)                                                    \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_LOG_TRACE(logger, __VA_ARGS__);                                                        \
        }                                                                                               \
    } while (false)

#define TBAI_LOG_DEBUG_THROTTLE(logger, period, ...)                                                    \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_LOG_DEBUG(logger, __VA_ARGS__);                                                        \
        }                                                                                               \
    } while (false)

#define TBAI_LOG_INFO_THROTTLE(logger, period, ...)                                                     \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_LOG_INFO(logger, __VA_ARGS__);                                                         \
        }                                                                                               \
    } while (false)

#define TBAI_LOG_WARN_THROTTLE(logger, period, ...)                                                     \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_LOG_WARN(logger, __VA_ARGS__);                                                         \
        }                                                                                               \
    } while (false)

#define TBAI_LOG_ERROR_THROTTLE(logger, period, ...)                                                    \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_LOG_ERROR(logger, __VA_ARGS__);                                                        \
        }                                                                                               \
    } while (false)

#define TBAI_LOG_FATAL_THROTTLE(logger, period, ...)                                                    \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_LOG_FATAL(logger, __VA_ARGS__);                                                        \
        }                                                                                               \
    } while (false)
namespace tbai {

std::shared_ptr<spdlog::logger> getLogger(const std::string &name);

}  // namespace tbai
