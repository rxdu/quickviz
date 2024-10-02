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
  Window win("Test Window", 1920, 1080);

  while (!win.ShouldClose()) {
    win.PollEvents();
    glEnable(GL_SCISSOR_TEST);
    glViewport(0, 0, 1919, 50);
    glScissor(0, 0, 1919, 50);
    glClearColor(1.0, 0.5, 0.2, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glViewport(0, 780, 1919, 300);
    glScissor(0, 780, 1919, 300);
    glClearColor(0.0, 0.0, 0.8, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);
    win.SwapBuffers();
  }

  return 0;
}
