#ifndef REMOTE_LOGGER_HPP
#define REMOTE_LOGGER_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>

/**
 * @brief A thread-safe logger that logs locally and sends logs to a remote server
 * 
 * This logger:
 * - Logs to ROS2 logger (local)
 * - Queues log messages for remote transmission
 * - Sends logs asynchronously via HTTP POST to a web server
 * - Non-blocking operation using background thread
 */
class RemoteLogger {
public:
    enum class Level {
        DEBUG,
        INFO,
        WARN,
        ERROR,
        FATAL
    };

    struct LogEntry {
        std::string source;
        Level level;
        std::string message;
        std::string timestamp;
    };

    /**
     * @brief Construct a new Remote Logger
     * 
     * @param node ROS2 node for local logging
     * @param remote_url URL endpoint for remote logging (e.g., "https://autogiro-test-api.noah.dev/api/logs")
     * @param source_name Source identifier (e.g., "wheelchair_code_module")
     * @param enable_remote Enable/disable remote logging (default: true)
     */
    RemoteLogger(
        rclcpp::Node::SharedPtr node,
        const std::string& remote_url,
        const std::string& source_name,
        bool enable_remote = true
    );

    ~RemoteLogger();

    // Logging methods
    void debug(const std::string& message);
    void info(const std::string& message);
    void warn(const std::string& message);
    void error(const std::string& message);
    void fatal(const std::string& message);

    // Formatted logging (printf-style)
    template<typename... Args>
    void debug(const char* format, Args... args) {
        log(Level::DEBUG, format_string(format, args...));
    }

    template<typename... Args>
    void info(const char* format, Args... args) {
        log(Level::INFO, format_string(format, args...));
    }

    template<typename... Args>
    void warn(const char* format, Args... args) {
        log(Level::WARN, format_string(format, args...));
    }

    template<typename... Args>
    void error(const char* format, Args... args) {
        log(Level::ERROR, format_string(format, args...));
    }

    template<typename... Args>
    void fatal(const char* format, Args... args) {
        log(Level::FATAL, format_string(format, args...));
    }

    // Control methods
    void enable_remote_logging(bool enable);
    void flush(); // Wait for all queued logs to be sent
    size_t get_queue_size() const;

private:
    void log(Level level, const std::string& message);
    void log_local(Level level, const std::string& message);
    void queue_remote_log(const LogEntry& entry);
    void worker_thread();
    bool send_log_to_server(const LogEntry& entry);
    std::string level_to_string(Level level) const;
    std::string get_current_timestamp() const;

    template<typename... Args>
    std::string format_string(const char* format, Args... args) {
        int size = std::snprintf(nullptr, 0, format, args...) + 1;
        if (size <= 0) return std::string(format);
        std::unique_ptr<char[]> buf(new char[size]);
        std::snprintf(buf.get(), size, format, args...);
        return std::string(buf.get(), buf.get() + size - 1);
    }

    rclcpp::Node::SharedPtr node_;
    std::string remote_url_;
    std::string source_name_;
    std::atomic<bool> enable_remote_;
    std::atomic<bool> shutdown_;

    std::queue<LogEntry> log_queue_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;

    static constexpr size_t MAX_QUEUE_SIZE = 1000;
};

#endif // REMOTE_LOGGER_HPP
