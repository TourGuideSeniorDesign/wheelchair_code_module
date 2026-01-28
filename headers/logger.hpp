#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"

/**
 * Logger - Comprehensive logging system that:
 * 1. Logs to ROS2 console
 * 2. Asynchronously sends logs to a web server via HTTP POST
 * 
 * Thread-safe and non-blocking.
 */
class Logger {
public:
    enum class Level {
        DEBUG,
        INFO,
        WARNING,
        ERROR
    };

    struct LogEntry {
        Level level;
        std::string message;
        std::chrono::system_clock::time_point timestamp;
    };

    /**
     * Initialize the logger with a ROS2 node and optional server URL
     * @param node ROS2 node for console logging
     * @param server_url URL to send logs to (default: https://autogiro-test-api.noah.dev/api/logs)
     * @param source Source identifier for logs (default: wheelchair_code_module)
     */
    static void init(rclcpp::Node::SharedPtr node, 
                    const std::string& server_url = "https://autogiro-test-api.noah.dev/api/logs",
                    const std::string& source = "wheelchair_code_module");

    /**
     * Shutdown the logger and cleanup resources
     */
    static void shutdown();

    /**
     * Log functions for different levels
     */
    static void debug(const std::string& message);
    static void info(const std::string& message);
    static void warning(const std::string& message);
    static void error(const std::string& message);

    /**
     * Generic log function
     */
    static void log(Level level, const std::string& message);

private:
    Logger() = default;
    ~Logger() = default;

    static Logger& instance();

    void init_internal(rclcpp::Node::SharedPtr node, 
                      const std::string& server_url,
                      const std::string& source);
    void shutdown_internal();
    void log_internal(Level level, const std::string& message);
    
    void worker_thread();
    void send_log_to_server(const LogEntry& entry);
    std::string level_to_string(Level level);
    std::string format_timestamp(const std::chrono::system_clock::time_point& tp);

    // ROS2 node for console logging
    rclcpp::Node::SharedPtr node_;
    
    // Server configuration
    std::string server_url_;
    std::string source_;
    
    // Thread-safe queue for log entries
    std::queue<LogEntry> log_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    
    // Worker thread
    std::thread worker_;
    std::atomic_bool running_{false};
    
    // Singleton flag
    static std::atomic_bool initialized_;
};

// Convenience macros
#define LOG_DEBUG(msg) Logger::debug(msg)
#define LOG_INFO(msg) Logger::info(msg)
#define LOG_WARNING(msg) Logger::warning(msg)
#define LOG_ERROR(msg) Logger::error(msg)

#endif // LOGGER_HPP
