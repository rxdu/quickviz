/*
 * @file app_log_handler.cpp
 * @date 12/17/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/component/logging/app_log_handler.hpp"

#include <string>
#include <sstream>
#include <iomanip>

namespace quickviz {
void AppLogHandler::Log(LogLevel level, const char* fmt, ...) {
  auto duration = Clock::now() - t0_;
  // Split into seconds and fractional nanoseconds
  auto seconds =
      std::chrono::duration_cast<std::chrono::seconds>(duration).count();
  auto microseconds =
      std::chrono::duration_cast<std::chrono::microseconds>(duration).count() %
      1'000'000;

  // Format into string
  std::ostringstream timestamp;
  timestamp << std::setfill('0') << std::setw(10) << seconds
            << "."  // Fixed-width seconds
            << std::setfill('0') << std::setw(6)
            << microseconds;  // Microseconds

  processor_.AddLog("[%s] [%s] %s\n", timestamp.str().c_str(),
                    level_str_map_[level], fmt);
}

void AppLogHandler::Draw() { processor_.Draw("AppLog"); }
}  // namespace quickviz