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

namespace quickviz {
class PointCloudToolPanel;

class InteractiveSceneManager : public GlSceneManager {
 public:
  InteractiveSceneManager(const std::string& name, Mode mode = Mode::k3D)
      : GlSceneManager(name, mode) {}

  void SetToolPanel(PointCloudToolPanel* panel) { tool_panel_ = panel; }

  void Draw() override;

 private:
  PointCloudToolPanel* tool_panel_ = nullptr;
};
}  // namespace quickviz

#endif  // QUICKVIZ_INTERACTIVE_SCENE_MANAGER_HPP