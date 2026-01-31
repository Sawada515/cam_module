/**
 * @file logger.hpp
 * @brief logger
 * @author sawada souta
 * @date 2026-01-24
 */

#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

inline void init_logger()
{
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        "/var/log/standard_assignment2/main.log", 10 * 1024 * 1024, 5
    );

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    spdlog::logger logger("main", {file_sink, console_sink});
    spdlog::set_default_logger(std::make_shared<spdlog::logger>(logger));
    spdlog::set_level(spdlog::level::info);
}

#endif
