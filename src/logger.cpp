#include "logger.hpp"
#include <curl/curl.h>
#include <iomanip>
#include <sstream>

std::atomic_bool Logger::initialized_{false};

Logger &Logger::instance() {
  static Logger instance;
  return instance;
}

void Logger::init(const std::string &source, const std::string &server_url,
                  bool enable_console) {
  instance().init_internal(source, server_url, enable_console);
}

void Logger::init_internal(const std::string &source,
                           const std::string &server_url, bool enable_console) {
  if (initialized_.exchange(true)) {
    return; // Already initialized
  }

  source_ = source;
  server_url_ = server_url;
  enable_console_ = enable_console;
  enable_remote_ = !server_url.empty();
  running_ = true;

  if (enable_remote_) {
    // Initialize libcurl globally
    curl_global_init(CURL_GLOBAL_DEFAULT);

    // Start worker thread
    worker_ = std::thread(&Logger::worker_thread, this);

    if (enable_console_) {
      std::lock_guard<std::mutex> lock(console_mutex_);
      std::cout << "[INFO] Logger initialized - sending to " << server_url_
                << std::endl;
    }
  } else if (enable_console_) {
    std::lock_guard<std::mutex> lock(console_mutex_);
    std::cout << "[INFO] Logger initialized - console only mode" << std::endl;
  }
}

void Logger::shutdown() { instance().shutdown_internal(); }

void Logger::shutdown_internal() {
  if (!initialized_) {
    return;
  }

  running_ = false;
  queue_cv_.notify_all();

  if (worker_.joinable()) {
    worker_.join();
  }

  if (enable_remote_) {
    curl_global_cleanup();
  }

  initialized_ = false;
}

void Logger::debug(const std::string &message) { log(Level::DEBUG, message); }

void Logger::info(const std::string &message) { log(Level::INFO, message); }

void Logger::warning(const std::string &message) {
  log(Level::WARNING, message);
}

void Logger::error(const std::string &message) { log(Level::ERROR, message); }

void Logger::log(Level level, const std::string &message) {
  instance().log_internal(level, message);
}

void Logger::log_internal(Level level, const std::string &message) {
  if (!initialized_) {
    // Fallback to stderr if not initialized
    std::cerr << "[" << level_to_string(level) << "] " << message << std::endl;
    return;
  }

  // Log to console
  if (enable_console_) {
    log_to_console(level, message);
  }

  // Queue for async sending to server (if enabled)
  if (enable_remote_) {
    LogEntry entry;
    entry.level = level;
    entry.message = message;
    entry.timestamp = std::chrono::system_clock::now();

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      log_queue_.push(entry);
    }
    queue_cv_.notify_one();
  }
}

void Logger::log_to_console(Level level, const std::string &message) {
  std::lock_guard<std::mutex> lock(console_mutex_);

  auto now = std::chrono::system_clock::now();
  auto timestamp = format_timestamp(now);

  // Use different streams for different levels
  std::ostream &out = (level == Level::ERROR || level == Level::WARNING)
                          ? std::cerr
                          : std::cout;

  out << "[" << timestamp << "] "
      << "[" << level_to_string(level) << "] " << message << std::endl;
}

void Logger::worker_thread() {
  while (running_) {
    LogEntry entry;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return !log_queue_.empty() || !running_; });

      if (!running_ && log_queue_.empty()) {
        break;
      }

      if (log_queue_.empty()) {
        continue;
      }

      entry = log_queue_.front();
      log_queue_.pop();
    }

    // Send to server (non-blocking - happens in background thread)
    send_log_to_server(entry);
  }

  // Drain remaining logs
  while (!log_queue_.empty()) {
    LogEntry entry;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (log_queue_.empty())
        break;
      entry = log_queue_.front();
      log_queue_.pop();
    }
    send_log_to_server(entry);
  }
}

// Callback for libcurl to discard response data
static size_t discard_callback(void *contents, size_t size, size_t nmemb,
                               void *userp) {
  return size * nmemb; // Pretend we handled all data
}

void Logger::send_log_to_server(const LogEntry &entry) {
  CURL *curl = curl_easy_init();
  if (!curl) {
    return;
  }

  // Build JSON payload
  std::ostringstream json;
  json << "{"
       << "\"source\":\"" << source_ << "\","
       << "\"level\":\"" << level_to_string(entry.level) << "\","
       << "\"message\":\"";

  // Escape special characters in message
  for (char c : entry.message) {
    switch (c) {
    case '"':
      json << "\\\"";
      break;
    case '\\':
      json << "\\\\";
      break;
    case '\b':
      json << "\\b";
      break;
    case '\f':
      json << "\\f";
      break;
    case '\n':
      json << "\\n";
      break;
    case '\r':
      json << "\\r";
      break;
    case '\t':
      json << "\\t";
      break;
    default:
      if (c < 0x20) {
        json << "\\u" << std::hex << std::setw(4) << std::setfill('0')
             << (int)c;
      } else {
        json << c;
      }
    }
  }

  json << "\","
       << "\"timestamp\":\"" << format_timestamp(entry.timestamp) << "\""
       << "}";

  std::string json_str = json.str();

  // Setup curl
  struct curl_slist *headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");

  curl_easy_setopt(curl, CURLOPT_URL, server_url_.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L); // 5 second timeout
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, discard_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, nullptr);

  // Perform the request (will timeout if server is unreachable)
  CURLcode res = curl_easy_perform(curl);

  // Silently ignore failures to avoid recursive logging
  (void)res; // Suppress unused variable warning

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
}

std::string Logger::level_to_string(Level level) {
  switch (level) {
  case Level::DEBUG:
    return "DEBUG";
  case Level::INFO:
    return "INFO";
  case Level::WARNING:
    return "WARNING";
  case Level::ERROR:
    return "ERROR";
  default:
    return "UNKNOWN";
  }
}

std::string
Logger::format_timestamp(const std::chrono::system_clock::time_point &tp) {
  auto time_t = std::chrono::system_clock::to_time_t(tp);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                tp.time_since_epoch()) %
            1000;

  std::ostringstream oss;
  oss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%S") << '.'
      << std::setfill('0') << std::setw(3) << ms.count() << 'Z';
  return oss.str();
}
