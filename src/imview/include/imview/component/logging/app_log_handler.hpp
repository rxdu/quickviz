/*
 * @file app_log_handler.hpp
 * @date 12/17/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef APP_LOG_HANDLER_HPP
#define APP_LOG_HANDLER_HPP

#include "imview/component/logging/log_processor.hpp"

#include <chrono>
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

  void Log(LogLevel level, const char* fmt, ...);
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