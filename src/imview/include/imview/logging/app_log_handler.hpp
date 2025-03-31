/*
 * @file app_log_handler.hpp
 * @date 12/17/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef APP_LOG_HANDLER_HPP
#define APP_LOG_HANDLER_HPP

#include "imview/logging/log_processor.hpp"

#include <chrono>
#include <sstream>
#include <iomanip>
#include <unordered_map>

namespace quickviz {
enum class LogLevel { kDebug, kInfo, kWarn, kError, kFatal };

class AppLogHandler {
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  AppLogHandler() { t0_ = Clock::now(); }
  ~AppLogHandler() = default;

 public:
  static AppLogHandler& GetInstance() {
    static AppLogHandler instance;
    return instance;
  }

  // void Log(LogLevel level, const char* fmt, ...);
  template <typename... Args>
  void Log(LogLevel level, const char* fmt, Args&&... args) {
    // generate timestamp
    auto duration = Clock::now() - t0_;
    auto seconds =
        std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    auto microseconds =
        std::chrono::duration_cast<std::chrono::microseconds>(duration)
            .count() %
        1'000'000;
    std::ostringstream timestamp;
    timestamp << std::setfill('0') << std::setw(10) << seconds
              << "."  // Fixed-width seconds
              << std::setfill('0') << std::setw(6)
              << microseconds;  // Microseconds

    // forward fixed arguments and variable arguments to AddLog
    processor_.AddLog("[%s][%s] ", timestamp.str().c_str(),
                      level_str_map_[level]);
    if constexpr (sizeof...(args) > 0) {
      processor_.AddLog(fmt, std::forward<Args>(args)...);
    } else {
      processor_.AddLog("%s", fmt);  // Treat fmt as a regular string
    }
    processor_.AddLog("\n");
  }

  void Draw();

 private:
  LogProcessor processor_;
  TimePoint t0_;

  std::unordered_map<LogLevel, const char*> level_str_map_ = {
      {LogLevel::kDebug, "DEBUG"},
      {LogLevel::kInfo, "INFO "},
      {LogLevel::kWarn, "WARN "},
      {LogLevel::kError, "ERROR"},
      {LogLevel::kFatal, "FATAL"}};
};
}  // namespace quickviz

#endif  // APP_LOG_HANDLER_HPP