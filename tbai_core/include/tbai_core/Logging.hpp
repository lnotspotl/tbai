#pragma once

#include <chrono>

#include <spdlog/spdlog.h>

#define TBAI_LOG_TRACE(...) SPDLOG_TRACE(__VA_ARGS__)
#define TBAI_LOG_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define TBAI_LOG_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define TBAI_LOG_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define TBAI_LOG_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define TBAI_LOG_FATAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

#define __TBAI_LOG_THROTTLE_CHECK(now, last, period) (last + period <= now || now < last)



#define TBAI_LOG_DEBUG_THROTTLE(period, ...)                                                            \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_LOG_DEBUG(__VA_ARGS__);                                                                \
        }                                                                                               \
    } while (false)

#define TBAI_LOG_INFO_THROTTLE(period, ...)                                                             \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_LOG_INFO(__VA_ARGS__);                                                                 \
        }                                                                                               \
    } while (false)

#define TBAI_LOG_WARN_THROTTLE(period, ...)                                                             \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_LOG_WARN(__VA_ARGS__);                                                                 \
        }                                                                                               \
    } while (false)

#define TBAI_LOG_ERROR_THROTTLE(period, ...)                                                            \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_LOG_ERROR(__VA_ARGS__);                                                                \
        }                                                                                               \
    } while (false)

#define TBAI_LOG_FATAL_THROTTLE(period, ...)                                                            \
    do {                                                                                                \
        thread_local double __log_throttle_last_hit__ = 0.0;                                            \
        double __log_throttle_now__ =                                                                   \
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); \
        if (__TBAI_LOG_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period)) {       \
            __log_throttle_last_hit__ = __log_throttle_now__;                                           \
            TBAI_LOG_FATAL(__VA_ARGS__);                                                                \
        }                                                                                               \
    } while (false)
