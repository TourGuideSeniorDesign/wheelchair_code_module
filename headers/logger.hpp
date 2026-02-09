#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

/**
 * Logger - Standalone logging utility that:
 * 1. Logs to console (stdout/stderr)
 * 2. Optionally sends logs asynchronously to a web server via HTTP POST
 *
 * Thread-safe and non-blocking.
 */
class Logger {
public:
  enum class Level { DEBUG, INFO, WARNING, ERROR };

  struct LogEntry {
    Level level;
    std::string message;
    std::chrono::system_clock::time_point timestamp;
  };

  /**
   * Initialize the logger with optional remote server configuration
   * @param source Source identifier for logs (default: "app")
   * @param server_url URL to send logs to (empty string disables remote
   * logging)
   * @param enable_console Enable/disable console output (default: true)
   */
  static void init(const std::string &source = "app",
                   const std::string &server_url = "",
                   bool enable_console = true);

  /**
   * Shutdown the logger and cleanup resources
   */
  static void shutdown();

  /**
   * Log functions for different levels
   */
  static void debug(const std::string &message);
  static void info(const std::string &message);
  static void warning(const std::string &message);
  static void error(const std::string &message);

  /**
   * Generic log function
   */
  static void log(Level level, const std::string &message);

private:
  Logger() = default;
  ~Logger() = default;

  static Logger &instance();

  void init_internal(const std::string &source, const std::string &server_url,
                     bool enable_console);
  void shutdown_internal();
  void log_internal(Level level, const std::string &message);
  void log_to_console(Level level, const std::string &message);

  void worker_thread();
  void send_log_to_server(const LogEntry &entry);
  std::string level_to_string(Level level);
  std::string format_timestamp(const std::chrono::system_clock::time_point &tp);

  // Configuration
  std::string server_url_;
  std::string source_;
  bool enable_console_{true};
  bool enable_remote_{false};

  // Thread-safe queue for log entries
  std::queue<LogEntry> log_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::mutex console_mutex_; // Protect console output

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
