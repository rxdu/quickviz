/*
 * @file main_docking_panel.hpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_MAIN_DOCKING_PANEL_HPP
#define QUICKVIZ_MAIN_DOCKING_PANEL_HPP

#include "imview/panel.hpp"

#include "panels/config_panel.hpp"
#include "panels/scene_panel.hpp"
#include "panels/console_panel.hpp"

namespace quickviz {
class MainDockingPanel : public Panel {
 public:
  MainDockingPanel(std::string name = "MainDockingPanel");

  void Draw() override;

 private:
  bool layout_initialized_ = false;

  ImGuiID dockspace_id_;
  ImGuiID config_panel_node_;
  ImGuiID scene_panel_node_;
  ImGuiID console_panel_node_;

  ConfigPanel config_panel_;
  ScenePanel scene_panel_;
  ConsolePanel console_panel_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_MAIN_DOCKING_PANEL_HPP