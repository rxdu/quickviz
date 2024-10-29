/*
 * test_cv_image_widget.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>

#include "imview/viewer.hpp"
#include "imview/widget/gl_scene_widget.hpp"

using namespace quickviz;

void RenderGL(const GlFrameBuffer& frame_buffer) {
  //  glBegin(GL_TRIANGLES);          // Each set of 3 vertices form a triangle
  //  glColor3f(0.0f, 0.0f, 1.0f); // Blue
  //  glVertex2f(0.1f, -0.6f);
  //  glVertex2f(0.7f, -0.6f);
  //  glVertex2f(0.4f, -0.1f);
  //
  //  glColor3f(1.0f, 0.0f, 0.0f); // Red
  //  glVertex2f(0.3f, -0.4f);
  //  glColor3f(0.0f, 1.0f, 0.0f); // Green
  //  glVertex2f(0.9f, -0.4f);
  //  glColor3f(0.0f, 0.0f, 1.0f); // Blue
  //  glVertex2f(0.6f, -0.9f);
  //  glEnd();
  glClearColor(1.0f, 0.0f, 0.0f, 1.0f); // Red background to make it noticeable
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

int main(int argc, char* argv[]) {
  Viewer viewer;

  auto gl_widget = std::make_shared<GlSceneWidget>("OpenGL Scene");
  gl_widget->OnResize(300, 200);
  gl_widget->SetPosition(0, 0);
  gl_widget->SetGlRenderFunction(RenderGL);
  viewer.AddSceneObject(gl_widget);

  viewer.Show();

  return 0;
}