/*
 * @file log_processor.cpp
 * @date 11/3/24
 * @brief adapted from imgui_demo.cpp
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/component/logging/log_processor.hpp"

namespace quickviz {
LogProcessor::LogProcessor() {
  auto_scroll_ = true;
  Clear();
}

void LogProcessor::Clear() {
  text_buffer_.clear();
  line_offsets_.clear();
  line_offsets_.push_back(0);
}

void LogProcessor::Draw(const char* title, bool* p_open) {
  ImGui::Spacing();

  // Options menu
  if (ImGui::BeginPopup("Options")) {
    ImGui::Checkbox("Auto-scroll", &auto_scroll_);
    ImGui::EndPopup();
  }

  // Main window
  if (ImGui::Button("Options")) ImGui::OpenPopup("Options");
  ImGui::SameLine();
  bool clear = ImGui::Button("Clear");
  ImGui::SameLine();
  bool copy = ImGui::Button("Copy");
  ImGui::SameLine();
  text_filter_.Draw("Filter", -100.0f);

  ImGui::Separator();

  if (ImGui::BeginChild("scrolling", ImVec2(0, 0), ImGuiChildFlags_None,
                        ImGuiWindowFlags_HorizontalScrollbar)) {
    if (clear) Clear();
    if (copy) ImGui::LogToClipboard();

    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
    const char* buf = text_buffer_.begin();
    const char* buf_end = text_buffer_.end();
    if (text_filter_.IsActive()) {
      // In this example we don't use the clipper when text_filter_ is enabled.
      // This is because we don't have random access to the result of our
      // filter. A real application processing logs with ten of thousands of
      // entries may want to store the result of search/filter.. especially
      // if the filtering function is not trivial (e.g. reg-exp).
      for (int line_no = 0; line_no < line_offsets_.Size; line_no++) {
        const char* line_start = buf + line_offsets_[line_no];
        const char* line_end = (line_no + 1 < line_offsets_.Size)
                                   ? (buf + line_offsets_[line_no + 1] - 1)
                                   : buf_end;
        if (text_filter_.PassFilter(line_start, line_end))
          ImGui::TextUnformatted(line_start, line_end);
      }
    } else {
      // The simplest and easy way to display the entire buffer:
      //   ImGui::TextUnformatted(buf_begin, buf_end);
      // And it'll just work. TextUnformatted() has specialization for large
      // blob of text and will fast-forward to skip non-visible lines. Here
      // we instead demonstrate using the clipper to only process lines that
      // are within the visible area. If you have tens of thousands of items
      // and their processing cost is non-negligible, coarse clipping them
      // on your side is recommended. Using ImGuiListClipper requires
      // - A) random access into your data
      // - B) items all being the  same height,
      // both of which we can handle since we have an array pointing to the
      // beginning of each line of text. When using the filter (in the block
      // of code above) we don't have random access into the data to display
      // anymore, which is why we don't use the clipper. Storing or skimming
      // through the search result would make it possible (and would be
      // recommended if you want to search through tens of thousands of
      // entries).
      ImGuiListClipper clipper;
      clipper.Begin(line_offsets_.Size);
      while (clipper.Step()) {
        for (int line_no = clipper.DisplayStart; line_no < clipper.DisplayEnd;
             line_no++) {
          const char* line_start = buf + line_offsets_[line_no];
          const char* line_end = (line_no + 1 < line_offsets_.Size)
                                     ? (buf + line_offsets_[line_no + 1] - 1)
                                     : buf_end;
          ImGui::TextUnformatted(line_start, line_end);
        }
      }
      clipper.End();
    }
    ImGui::PopStyleVar();

    // Keep up at the bottom of the scroll region if we were already at the
    // bottom at the beginning of the frame. Using a scrollbar or
    // mouse-wheel will take away from the bottom edge.
    if (auto_scroll_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
      ImGui::SetScrollHereY(1.0f);
  }
  ImGui::EndChild();
  //      ImGui::End();
}
}  // namespace quickviz