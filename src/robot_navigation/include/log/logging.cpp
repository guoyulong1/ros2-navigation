#include "logging.hpp"
#include <filesystem>
#include <iostream>


// 定义logger和init_logger()
std::shared_ptr<spdlog::logger> logger;

void init_logger() {
    if (LOG_ENABLED) {
        try {
            // 确保日志目录存在
            std::filesystem::path log_path(LOG_FILE_PATH);
            if (log_path.has_parent_path()) {
                std::filesystem::create_directories(log_path.parent_path());
            }
            auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(LOG_FILE_PATH, true);
            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

            // 设置格式模式 - 使用正确的格式说明符
            file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v");
            console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%#] %v");

            // 创建logger
            logger = std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list{file_sink, console_sink});
            
            // 设置日志级别
            logger->set_level(LOG_LEVEL);
            
            // 设置刷新策略（可选，确保日志及时写入文件）
            logger->flush_on(spdlog::level::info);

            std::cout << "Logger initialized successfully. Log file: " << LOG_FILE_PATH << std::endl;
        } catch (const spdlog::spdlog_ex &ex) {
            std::cerr << "Log initialization failed: " << ex.what() << std::endl;
        }
    }else{
        std::cout << "Logging is disabled." << std::endl;
    }
}
