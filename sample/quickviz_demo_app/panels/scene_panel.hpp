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

#include "imview/component/opengl/renderer/grid.hpp"
#include "imview/component/opengl/gl_scene_manager.hpp"

namespace quickviz {
class ScenePanel : public GlSceneManager {
 public:
  ScenePanel(const std::string& panel_name);
};
}  // namespace quickviz

#endif  // QUICKVIZ_SCENE_PANEL_HPP