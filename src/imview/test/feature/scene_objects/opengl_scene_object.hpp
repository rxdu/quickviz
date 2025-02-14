/*
 * @file opengl_scene_object.hpp
 * @date 10/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_OPENGL_SCENE_OBJECT_HPP
#define QUICKVIZ_OPENGL_SCENE_OBJECT_HPP

#include "glad/glad.h"
#include "imview/panel.hpp"

namespace quickviz {
class OpenGLSceneObject : public Panel {
 public:
  OpenGLSceneObject(std::string name, float r = 1.0, float g = 0.0,
                    float b = 0.0)
      : Panel(name), r(r), g(g), b(b) {};

  void Draw() override {
    glEnable(GL_SCISSOR_TEST);
    glViewport(x_, y_, width_, height_);
    glScissor(x_, y_, width_, height_);

    glClearColor(r, g, b, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glDisable(GL_SCISSOR_TEST);
  }

  float r, g, b;
};
}  // namespace quickviz

#endif  // QUICKVIZ_OPENGL_SCENE_OBJECT_HPP
