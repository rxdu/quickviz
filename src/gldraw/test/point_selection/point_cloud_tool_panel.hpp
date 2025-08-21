/*
 * @file point_cloud_tool_panel.hpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_POINT_CLOUD_TOOL_PANEL_HPP
#define QUICKVIZ_POINT_CLOUD_TOOL_PANEL_HPP

#include "imview/panel.hpp"
#include "gldraw/gl_scene_manager.hpp"

namespace quickviz {
class PointCloudToolPanel : public Panel {
 public:
  PointCloudToolPanel(const std::string& name, GlSceneManager* scene_manager)
      : Panel(name), scene_manager_(scene_manager) {}

  void Draw() override;

  struct MouseInfo {
    bool valid = false;
    glm::vec2 screen_pos;
    GlSceneManager::MouseRay ray;
  };

  void UpdateMouseInfo(const MouseInfo& info) { mouse_info_ = info; }

 private:
  GlSceneManager* scene_manager_ = nullptr;
  MouseInfo mouse_info_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_POINT_CLOUD_TOOL_PANEL_HPP