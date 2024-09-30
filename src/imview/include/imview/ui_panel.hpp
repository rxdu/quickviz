/*
 * @file ui_panel.hpp
 * @date 10/1/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_UI_PANEL_HPP
#define QUICKVIZ_UI_PANEL_HPP

#include "imview/panel.hpp"

namespace quickviz {
class UiPanel : public Panel {
 public:
  UiPanel(std::string name = "UiPanel");

  void SetPosition(float x, float y) override;
  void OnResize(float width, float height) override;
  void OnRender() override;

 private:
  void StartNewFrame();
  void RenderFrame();
};
}  // namespace quickviz

#endif  // QUICKVIZ_UI_PANEL_HPP