/*
 * @file editor_state.cpp
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "editor_state.hpp"

#include <algorithm>

#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/selection_manager.hpp"
#include "gldraw/tools/point_selection_tool.hpp"

namespace quickviz::editor {

void EditorState::Initialize(PointCloud* cloud,
                             PointSelectionTool* tool,
                             std::vector<glm::vec3> points,
                             std::vector<glm::vec3> colors) {
  cloud_ = cloud;
  tool_ = tool;
  points_ = std::move(points);
  colors_ = std::move(colors);
  if (colors_.size() != points_.size()) {
    colors_.assign(points_.size(), glm::vec3(0.7f, 0.7f, 0.7f));
  }
  alive_.assign(points_.size(), true);
  live_count_ = points_.size();
  RebuildVisibleCloud();
}

void EditorState::KillPoints(const std::vector<std::size_t>& indices) {
  if (!cloud_ || indices.empty()) return;
  for (auto idx : indices) {
    if (idx < alive_.size() && alive_[idx]) {
      alive_[idx] = false;
      --live_count_;
    }
  }
  if (tool_) {
    tool_->ClearSelection();
  }
  RebuildVisibleCloud();
}

void EditorState::RevivePoints(const std::vector<std::size_t>& indices) {
  if (!cloud_ || indices.empty()) return;
  for (auto idx : indices) {
    if (idx < alive_.size() && !alive_[idx]) {
      alive_[idx] = true;
      ++live_count_;
    }
  }
  // Visible-index -> original-index mapping changes on rebuild, so any
  // selection the tool currently holds becomes stale. Clear it.
  if (tool_) {
    tool_->ClearSelection();
  }
  RebuildVisibleCloud();
}

std::vector<std::size_t> EditorState::CurrentSelectionIndices() const {
  std::vector<std::size_t> out;
  if (!tool_) return out;
  const auto& multi = tool_->GetMultiSelection();
  const auto points = multi.GetPoints();
  out.reserve(points.size());
  for (const auto& sel : points) {
    if (sel.cloud_name != kEditableCloudName) continue;
    if (sel.point_index >= visible_to_original_.size()) continue;
    out.push_back(visible_to_original_[sel.point_index]);
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

std::size_t EditorState::SelectedPointCount() const {
  if (!tool_) return 0;
  return tool_->GetMultiSelection().Count();
}

void EditorState::RebuildVisibleCloud() {
  if (!cloud_) return;

  std::vector<glm::vec3> visible_points;
  std::vector<glm::vec3> visible_colors;
  visible_points.reserve(live_count_);
  visible_colors.reserve(live_count_);
  visible_to_original_.clear();
  visible_to_original_.reserve(live_count_);

  for (std::size_t i = 0; i < points_.size(); ++i) {
    if (!alive_[i]) continue;
    visible_points.push_back(points_[i]);
    visible_colors.push_back(colors_[i]);
    visible_to_original_.push_back(i);
  }

  cloud_->SetPoints(std::move(visible_points), std::move(visible_colors));
}

}  // namespace quickviz::editor
