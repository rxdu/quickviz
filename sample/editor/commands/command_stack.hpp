/*
 * @file command_stack.hpp
 * @brief Bounded undo/redo stack for editor commands
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EDITOR_COMMAND_STACK_HPP
#define QUICKVIZ_EDITOR_COMMAND_STACK_HPP

#include <cstddef>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "command.hpp"

namespace quickviz::editor {

class CommandStack {
 public:
  explicit CommandStack(std::size_t max_depth = 256);

  // Run command and push it onto the done stack. Drops the redo stack.
  void Exec(std::unique_ptr<Command> cmd);

  bool Undo();
  bool Redo();

  bool CanUndo() const { return !done_.empty(); }
  bool CanRedo() const { return !undone_.empty(); }

  std::size_t DoneCount() const { return done_.size(); }
  std::size_t UndoneCount() const { return undone_.size(); }

  // Snapshots of descriptions, ordered oldest-first for done, newest-first for
  // undone (i.e. [Top of undo] ... [Bottom of redo]).
  std::vector<std::string> DoneDescriptions() const;
  std::vector<std::string> UndoneDescriptions() const;

  std::string TopUndoDescription() const;
  std::string TopRedoDescription() const;

  void Clear();

 private:
  std::deque<std::unique_ptr<Command>> done_;
  std::deque<std::unique_ptr<Command>> undone_;
  std::size_t max_depth_;
};

}  // namespace quickviz::editor

#endif  // QUICKVIZ_EDITOR_COMMAND_STACK_HPP
