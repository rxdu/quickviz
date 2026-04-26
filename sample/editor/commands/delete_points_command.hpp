/*
 * @file delete_points_command.hpp
 * @brief Undoable deletion of selected points
 *
 * The captured indices are *original-array* indices — the indices the editor
 * uses internally — not visible-cloud indices. Capture happens at command
 * construction, before Do() runs.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EDITOR_DELETE_POINTS_COMMAND_HPP
#define QUICKVIZ_EDITOR_DELETE_POINTS_COMMAND_HPP

#include <cstddef>
#include <string>
#include <vector>

#include "command.hpp"

namespace quickviz::editor {

class EditorState;

class DeletePointsCommand : public Command {
 public:
  DeletePointsCommand(EditorState* state,
                      std::vector<std::size_t> original_indices);

  void Do() override;
  void Undo() override;
  std::string Description() const override;

 private:
  EditorState* state_;
  std::vector<std::size_t> indices_;
};

}  // namespace quickviz::editor

#endif  // QUICKVIZ_EDITOR_DELETE_POINTS_COMMAND_HPP
