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
#include "component/log_processor.hpp"

namespace quickviz {
class ConsolePanel : public Panel {
 public:
  ConsolePanel(std::string name = "Debug");

  void Draw() override;

 private:
  LogProcessor log_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_CONSOLE_PANEL_HPP