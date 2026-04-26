/*
 * @file console_panel.hpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CONSOLE_PANEL_HPP
#define QUICKVIZ_CONSOLE_PANEL_HPP

#include "imview/panel.hpp"

namespace quickviz {
class ConsolePanel : public Panel {
 public:
  ConsolePanel(std::string name = "Console");

  void Draw() override;
};
}  // namespace quickviz

#endif  // QUICKVIZ_CONSOLE_PANEL_HPP