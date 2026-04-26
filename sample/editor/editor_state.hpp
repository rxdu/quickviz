/*
 * @file editor_state.hpp
 * @brief In-memory editing state for the editor sample
 *
 * Holds the loaded point cloud data as the source of truth and projects the
 * "alive" subset into the library's PointCloud renderable. All editing
 * operations mutate this state via Commands; the renderable is a view.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EDITOR_STATE_HPP
#define QUICKVIZ_EDITOR_STATE_HPP

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "commands/command_stack.hpp"

namespace quickviz {
class PointCloud;
class PointSelectionTool;
}  // namespace quickviz

namespace quickviz::editor {

// Name under which the editable cloud is registered in the SceneManager.
inline constexpr const char* kEditableCloudName = "edit_cloud";

class EditorState {
 public:
  EditorState() = default;
  ~EditorState() = default;

  EditorState(const EditorState&) = delete;
  EditorState& operator=(const EditorState&) = delete;

  // Wire the state to the live renderable + selection tool. Both pointers
  // are non-owning; the SceneManager and the tool's shared_ptr in main()
  // outlive the state. Loads the original data into the alive set.
  void Initialize(PointCloud* cloud,
                  PointSelectionTool* tool,
                  std::vector<glm::vec3> points,
                  std::vector<glm::vec3> colors);

  // Editing primitives (called by commands). Both are no-ops on empty input.
  // Indices refer to positions in the original loaded array.
  void KillPoints(const std::vector<std::size_t>& indices);
  void RevivePoints(const std::vector<std::size_t>& indices);

  // Currently selected indices in the *original* point array, snapshot from
  // PointSelectionTool's multi-selection. The selection tool reports indices
  // into the live (visible) cloud; this method maps them back through
  // visible_to_original_ before returning.
  std::vector<std::size_t> CurrentSelectionIndices() const;

  std::size_t TotalPointCount() const { return points_.size(); }
  std::size_t LivePointCount() const { return live_count_; }
  std::size_t SelectedPointCount() const;

  bool IsInitialized() const { return cloud_ != nullptr; }

  CommandStack& History() { return history_; }
  const CommandStack& History() const { return history_; }

  PointSelectionTool* Tool() { return tool_; }

 private:
  void RebuildVisibleCloud();

  PointCloud* cloud_ = nullptr;          // non-owning
  PointSelectionTool* tool_ = nullptr;   // non-owning

  // Source-of-truth, indexed by original-array index.
  std::vector<glm::vec3> points_;
  std::vector<glm::vec3> colors_;
  std::vector<bool> alive_;
  std::size_t live_count_ = 0;

  // Maps visible-cloud index -> original-array index. Rebuilt every time the
  // visible cloud is regenerated. SelectionManager hands us visible indices.
  std::vector<std::size_t> visible_to_original_;

  CommandStack history_;
};

}  // namespace quickviz::editor

#endif  // QUICKVIZ_EDITOR_STATE_HPP
