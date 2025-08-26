/*
 * @file interactive_scene_manager.hpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_INTERACTIVE_SCENE_MANAGER_HPP
#define QUICKVIZ_INTERACTIVE_SCENE_MANAGER_HPP

#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "visualization/selection/point_cloud_selector.hpp"
#include <memory>

namespace quickviz {
class PointCloudToolPanel;

class InteractiveSceneManager : public GlSceneManager {
 public:
  InteractiveSceneManager(const std::string& name, Mode mode = Mode::k3D)
      : GlSceneManager(name, mode) {}

  void SetToolPanel(PointCloudToolPanel* panel) { tool_panel_ = panel; }

  // Point cloud selection integration
  void SetPointCloud(std::shared_ptr<PointCloud> point_cloud);
  visualization::PointCloudSelector* GetSelector() const { return selector_.get(); }
  std::shared_ptr<PointCloud> GetPointCloud() const { return point_cloud_; }
  
  void Draw() override;

 private:
  PointCloudToolPanel* tool_panel_ = nullptr;
  
  // Point cloud selection
  std::shared_ptr<PointCloud> point_cloud_;
  std::unique_ptr<visualization::PointCloudSelector> selector_;
  bool selection_enabled_ = true;
  
  // Internal methods for handling input
  void HandleMouseInput();
  void HandleKeyboardInput();
};
}  // namespace quickviz

#endif  // QUICKVIZ_INTERACTIVE_SCENE_MANAGER_HPP