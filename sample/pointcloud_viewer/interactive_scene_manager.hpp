/*
 * @file interactive_scene_manager.hpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_INTERACTIVE_SCENE_MANAGER_HPP
#define QUICKVIZ_INTERACTIVE_SCENE_MANAGER_HPP

#include "gldraw/gl_scene_panel.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/tools/point_selection_tool.hpp"
#include <memory>

namespace quickviz {
class PointCloudToolPanel;

class InteractiveSceneManager : public GlScenePanel {
 public:
  InteractiveSceneManager(const std::string& name, SceneManager::Mode mode = SceneManager::Mode::k3D)
      : GlScenePanel(name, mode) {
    // Disable built-in selection in SceneInputHandler to avoid conflicts with PointSelectionTool
    // but keep camera controls and tool input forwarding active
    GetSceneInputHandler()->SetSelectionEnabled(false);
    
    InitializeTools();
  }

  void SetToolPanel(PointCloudToolPanel* panel) { tool_panel_ = panel; }

  // Point cloud setup  
  void SetPointCloud(std::unique_ptr<PointCloud> point_cloud);
  
  void Draw() override;

 private:
  PointCloudToolPanel* tool_panel_ = nullptr;
  
  // UI components
  bool selection_enabled_ = true;
  
  // Tool management
  std::shared_ptr<PointSelectionTool> point_selection_tool_;
  
  // Internal methods
  void InitializeTools();
  void HandleMouseInput();
  void HandleKeyboardInput();
};
}  // namespace quickviz

#endif  // QUICKVIZ_INTERACTIVE_SCENE_MANAGER_HPP