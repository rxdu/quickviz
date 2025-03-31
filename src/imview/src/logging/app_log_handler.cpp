/*
 * @file app_log_handler.cpp
 * @date 12/17/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/logging/app_log_handler.hpp"

namespace quickviz {
void AppLogHandler::Draw() { processor_.Draw("AppLog"); }
}  // namespace quickviz