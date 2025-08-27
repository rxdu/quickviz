/*
 * @file interactive_scene_manager.hpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_INTERACTIVE_SCENE_MANAGER_HPP
#define QUICKVIZ_INTERACTIVE_SCENE_MANAGER_HPP

#include "gldraw/scene_view_panel.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include <memory>

namespace quickviz {
class PointCloudToolPanel;

class InteractiveSceneManager : public SceneViewPanel {
 public:
  InteractiveSceneManager(const std::string& name, GlSceneManager::Mode mode = GlSceneManager::Mode::k3D)
      : SceneViewPanel(name, mode) {}

  void SetToolPanel(PointCloudToolPanel* panel) { tool_panel_ = panel; }

  // Point cloud setup  
  void SetPointCloud(std::unique_ptr<PointCloud> point_cloud);
  
  void Draw() override;

 private:
  PointCloudToolPanel* tool_panel_ = nullptr;
  
  // UI components
  bool selection_enabled_ = true;
  
  // Internal methods for handling input
  void HandleMouseInput();
  void HandleKeyboardInput();
};
}  // namespace quickviz

#endif  // QUICKVIZ_INTERACTIVE_SCENE_MANAGER_HPP