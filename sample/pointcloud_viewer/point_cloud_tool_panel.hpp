/*
 * @file point_cloud_tool_panel.hpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_POINT_CLOUD_TOOL_PANEL_HPP
#define QUICKVIZ_POINT_CLOUD_TOOL_PANEL_HPP

#include "viewer/panel.hpp"
#include "scene/gl_scene_panel.hpp"
#include "scene/tools/point_selection_tool.hpp"

namespace quickviz {
class InteractiveSceneManager;

class PointCloudToolPanel : public Panel {
 public:
  PointCloudToolPanel(const std::string& name, InteractiveSceneManager* scene_manager)
      : Panel(name), scene_manager_(scene_manager) {}

  void Draw() override;

  struct MouseInfo {
    bool valid = false;
    glm::vec2 screen_pos;
    glm::vec3 world_pos; // 3D position if available from GPU selection
  };

  void UpdateMouseInfo(const MouseInfo& info) { mouse_info_ = info; }

 private:
  InteractiveSceneManager* scene_manager_ = nullptr;
  MouseInfo mouse_info_;
  float point_size_ = 3.0f;  // Default point size
  
  // Tool selection state
  bool point_selection_tool_active_ = false;
  int selection_radius_ = 3;
  int selection_mode_index_ = 0;  // Index for combo box
  
  // Helper methods
  InteractiveSceneManager* GetInteractiveSceneManager() const;
  void DrawToolSelectionUI();
  void DrawPointSelectionControls();
  std::shared_ptr<PointSelectionTool> GetPointSelectionTool() const;
};
}  // namespace quickviz

#endif  // QUICKVIZ_POINT_CLOUD_TOOL_PANEL_HPP