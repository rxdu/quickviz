/*
 * test_wgui.cpp
 *
 * Created on: Jul 22, 2021 14:50
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "glad/glad.h"
#include "imview/window.hpp"
#include "imview/component/opengl/renderer/grid.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  int width = 1920;
  int height = 1080;
  Window win("Test Window", width, height);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Orthographic projection for a top-down view
  float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);
  glm::mat4 projection =
      glm::perspective(glm::radians(45.0f), aspect_ratio, 0.1f, 100.0f);

  // Simple view matrix looking at an angle
  glm::mat4 view = glm::lookAt(
      glm::vec3(10.0f, 10.0f, 10.0f),  // Camera positioned at an angle
      glm::vec3(0.0f, 0.0f, 0.0f),     // Looking at the origin
      glm::vec3(0.0f, 1.0f, 0.0f)      // Up vector pointing along the Y-axis
  );

  // set up grid
  Grid grid(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));

  while (!win.ShouldClose()) {
    win.PollEvents();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    grid.OnDraw(projection, view);

    win.SwapBuffers();
  }

  return 0;
}
