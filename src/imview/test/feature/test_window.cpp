/*
 * test_wgui.cpp
 *
 * Created on: Jul 22, 2021 14:50
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>

#include "glad/glad.h"
#include "imview/window.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  int width = 1920;
  int height = 1080;
  Window win("Test Window", width, height);

  while (!win.ShouldClose()) {
    win.PollEvents();

    glEnable(GL_SCISSOR_TEST);

    glViewport(0, 0, width, height / 3.0);
    glScissor(0, 0, width, height / 3.0);
    glClearColor(0.8, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glViewport(0, height / 3.0, width, height / 3.0);
    glScissor(0, height / 3.0, width, height / 3.0);
    glClearColor(0.0, 0.8, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glViewport(0, height * 2 / 3.0, width, height / 3.0);
    glScissor(0, height * 2 / 3.0, width, height / 3.0);
    glClearColor(0.0, 0.0, 0.8, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glDisable(GL_SCISSOR_TEST);

    win.SwapBuffers();
  }

  return 0;
}
