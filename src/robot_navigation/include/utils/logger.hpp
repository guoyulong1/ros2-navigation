#pragma once
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

// 配置日志路径、日志级别和是否启用日志
#define LOG_FILE_PATH "log/navigation.log"  // 可以在这里修改日志文件的路径
#define LOG_LEVEL spdlog::level::info  // 可以设置为 info, warn, debug 等
#define LOG_ENABLED true  // 是否启用日志

// 声明logger变量
extern std::shared_ptr<spdlog::logger> logger;

void init_logger();

// 方案：分离格式字符串和参数
#define LOG_INFO(fmt, ...)  logger->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::info, fmt, ##__VA_ARGS__); 
#define LOG_WARN(fmt, ...)  logger->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::warn, fmt, ##__VA_ARGS__); 
#define LOG_ERROR(fmt, ...) logger->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::err, fmt, ##__VA_ARGS__); 