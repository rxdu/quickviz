/*
 * @file command_stack.cpp
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "command_stack.hpp"

namespace quickviz::editor {

CommandStack::CommandStack(std::size_t max_depth) : max_depth_(max_depth) {}

void CommandStack::Exec(std::unique_ptr<Command> cmd) {
  if (!cmd) return;
  cmd->Do();
  done_.push_back(std::move(cmd));
  undone_.clear();
  while (done_.size() > max_depth_) {
    done_.pop_front();
  }
}

bool CommandStack::Undo() {
  if (done_.empty()) return false;
  auto cmd = std::move(done_.back());
  done_.pop_back();
  cmd->Undo();
  undone_.push_back(std::move(cmd));
  return true;
}

bool CommandStack::Redo() {
  if (undone_.empty()) return false;
  auto cmd = std::move(undone_.back());
  undone_.pop_back();
  cmd->Do();
  done_.push_back(std::move(cmd));
  return true;
}

std::vector<std::string> CommandStack::DoneDescriptions() const {
  std::vector<std::string> out;
  out.reserve(done_.size());
  for (const auto& c : done_) out.push_back(c->Description());
  return out;
}

std::vector<std::string> CommandStack::UndoneDescriptions() const {
  std::vector<std::string> out;
  out.reserve(undone_.size());
  // Walk from most-recently-undone to oldest, so the redo panel reads
  // top-to-bottom in undo order.
  for (auto it = undone_.rbegin(); it != undone_.rend(); ++it) {
    out.push_back((*it)->Description());
  }
  return out;
}

std::string CommandStack::TopUndoDescription() const {
  return done_.empty() ? "" : done_.back()->Description();
}

std::string CommandStack::TopRedoDescription() const {
  return undone_.empty() ? "" : undone_.back()->Description();
}

void CommandStack::Clear() {
  done_.clear();
  undone_.clear();
}

}  // namespace quickviz::editor
