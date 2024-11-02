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

#include "imview/widget/gl_widget.hpp"

#include "panels/menu_bar.hpp"
#include "panels/config_panel.hpp"
#include "panels/console_panel.hpp"

namespace quickviz {
class MainDockingPanel : public Panel {
 public:
  MainDockingPanel(std::string name = "MainDockingPanel");

  void Draw() override;

  void ChangeDebugPanelVisibility(bool visible);

 private:

  bool layout_initialized_ = false;
  ImGuiID dockspace_id_;

  ImGuiID config_panel_node_;
  ImGuiID console_panel_node_;
  ImGuiID gl_scene_widget_node_;

  ConfigPanel config_panel_{"Config"};
  ConsolePanel console_panel_{"Console"};
  GlSceneWidget gl_scene_widget_{"Scene"};
};
}  // namespace quickviz

#endif  // QUICKVIZ_MAIN_DOCKING_PANEL_HPP