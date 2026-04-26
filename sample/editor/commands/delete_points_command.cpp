/*
 * @file delete_points_command.cpp
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "delete_points_command.hpp"

#include "../editor_state.hpp"

namespace quickviz::editor {

DeletePointsCommand::DeletePointsCommand(
    EditorState* state, std::vector<std::size_t> original_indices)
    : state_(state), indices_(std::move(original_indices)) {}

void DeletePointsCommand::Do() {
  if (state_) state_->KillPoints(indices_);
}

void DeletePointsCommand::Undo() {
  if (state_) state_->RevivePoints(indices_);
}

std::string DeletePointsCommand::Description() const {
  return "Delete " + std::to_string(indices_.size()) + " point" +
         (indices_.size() == 1 ? "" : "s");
}

}  // namespace quickviz::editor
