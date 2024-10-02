/*
 * @file opengl_panel.hpp
 * @date 10/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_OPENGL_PANEL_HPP
#define QUICKVIZ_OPENGL_PANEL_HPP

#include "glad/glad.h"
#include "imview/interface/renderable.hpp"

namespace quickviz {
class OpenGLPanel : public Renderable {
 public:
  OpenGLPanel(float r = 1.0, float g = 0.5, float b = 0.2)
      : r(r), g(g), b(b) {};

  bool IsVisible() const override { return true; }
  bool IsContainer() const override { return false; }
  void OnRender() override {
    glClearColor(r, g, b, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
  }

  float r, g, b;
};
}  // namespace quickviz

#endif  // QUICKVIZ_OPENGL_PANEL_HPP
