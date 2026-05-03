/*
 * @file log_processor.hpp
 * @date 11/3/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_LOG_PROCESSOR_HPP
#define QUICKVIZ_LOG_PROCESSOR_HPP

#include "imgui.h"

namespace quickviz {
class LogProcessor {
 public:
  LogProcessor();

  // public methods
  void Clear();

  void AddLog(const char* fmt, ...) IM_FMTARGS(2) {
    int old_size = text_buffer_.size();
    va_list args;
    va_start(args, fmt);
    text_buffer_.appendfv(fmt, args);
    va_end(args);
    for (int new_size = text_buffer_.size(); old_size < new_size; old_size++)
      if (text_buffer_[old_size] == '\n') line_offsets_.push_back(old_size + 1);
  }

  void Draw(const char* title, bool* p_open = NULL);

 private:
  ImGuiTextBuffer text_buffer_;
  ImGuiTextFilter text_filter_;
  ImVector<int> line_offsets_;  // Index to lines offset. We maintain this with
                                // AddLog() calls.
  bool auto_scroll_;            // Keep scrolling if already at the bottom.
};
}  // namespace quickviz

#endif  // QUICKVIZ_LOG_PROCESSOR_HPP