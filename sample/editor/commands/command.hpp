/*
 * @file command.hpp
 * @brief Editor-side Command pattern (sample-private, not part of the library)
 *
 * The library is visualization-first; commands and history live here in the
 * editor sample. This file deliberately avoids any dependency on src/.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EDITOR_COMMAND_HPP
#define QUICKVIZ_EDITOR_COMMAND_HPP

#include <memory>
#include <string>

namespace quickviz::editor {

class Command {
 public:
  virtual ~Command() = default;

  virtual void Do() = 0;
  virtual void Undo() = 0;

  virtual std::string Description() const = 0;

  Command(const Command&) = delete;
  Command& operator=(const Command&) = delete;

 protected:
  Command() = default;
};

}  // namespace quickviz::editor

#endif  // QUICKVIZ_EDITOR_COMMAND_HPP
