/*
 * @file scene_panel.hpp
 * @date 11/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SCENE_PANEL_HPP
#define QUICKVIZ_SCENE_PANEL_HPP

#include <memory>

#include "imview/widget/gl_widget.hpp"
#include "imview/component/opengl/grid.hpp"
#include "imview/component/opengl/camera.hpp"
#include "imview/component/opengl/camera_controller.hpp"

namespace quickviz {
class ScenePanel : public GlWidget {
  enum MouseButton {
    kLeft = 0,
    kRight = 1,
    kMiddle = 2,
  };

 public:
  ScenePanel(const std::string& panel_name);

  void Draw() override;

 private:
  std::unique_ptr<Camera> camera_;
  std::unique_ptr<CameraController> camera_controller_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_SCENE_PANEL_HPP