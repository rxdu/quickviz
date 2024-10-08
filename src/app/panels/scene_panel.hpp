/*
 * @file scene_panel.hpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SCENE_PANEL_HPP
#define QUICKVIZ_SCENE_PANEL_HPP

#include "imview/panel.hpp"

namespace quickviz {
class ScenePanel : public Panel {
 public:
  ScenePanel(std::string name = "Scene");

  void Draw() override;
};
}  // namespace quickviz

#endif  // QUICKVIZ_SCENE_PANEL_HPP