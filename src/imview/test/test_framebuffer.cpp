/*
 * test_wgui.cpp
 *
 * Created on: Jul 22, 2021 14:50
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>

#include "glad/glad.h"
#include "imview/window.hpp"
#include "imview/widget/details/gl_frame_buffer.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  int width = 1920;
  int height = 1080;
  Window win("Test Window", width, height);

  // Create a framebuffer
  auto frame_buffer = std::make_unique<GlFrameBuffer>(width, height);

  while (!win.ShouldClose()) {
    win.PollEvents();

    // Bind and clear the framebuffer
    frame_buffer->Bind();
    glViewport(0, 0, width, height);  // Set the viewport to framebuffer size
    glClearColor(1.0f, 0.0f, 0.0f, 1.0f);  // Clear with red
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    frame_buffer->Unbind();

    // Render the framebuffer's texture directly to the screen
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  // Bind default framebuffer
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);  // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, frame_buffer->GetTextureId());
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(-1.0f, -1.0f);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(1.0f, -1.0f);
    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(1.0f, 1.0f);
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(-1.0f, 1.0f);
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0);

    win.SwapBuffers();
  }

  return 0;
}
