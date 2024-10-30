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
  frame_buffer.Clear();

  // Define shapes enclosed within a pair of glBegin and glEnd
  glBegin(GL_QUADS);            // Each set of 4 vertices form a quad
  glColor3f(1.0f, 0.0f, 0.0f);  // Red
  glVertex2f(-0.8f, 0.1f);  // Define vertices in counter-clockwise (CCW) order
  glVertex2f(-0.2f, 0.1f);  //  so that the normal (front-face) is facing you
  glVertex2f(-0.2f, 0.7f);
  glVertex2f(-0.8f, 0.7f);

  glColor3f(0.0f, 1.0f, 0.0f);  // Green
  glVertex2f(-0.7f, -0.6f);
  glVertex2f(-0.1f, -0.6f);
  glVertex2f(-0.1f, 0.0f);
  glVertex2f(-0.7f, 0.0f);

  glColor3f(0.2f, 0.2f, 0.2f);  // Dark Gray
  glVertex2f(-0.9f, -0.7f);
  glColor3f(1.0f, 1.0f, 1.0f);  // White
  glVertex2f(-0.5f, -0.7f);
  glColor3f(0.2f, 0.2f, 0.2f);  // Dark Gray
  glVertex2f(-0.5f, -0.3f);
  glColor3f(1.0f, 1.0f, 1.0f);  // White
  glVertex2f(-0.9f, -0.3f);
  glEnd();

  glBegin(GL_TRIANGLES);        // Each set of 3 vertices form a triangle
  glColor3f(0.0f, 0.0f, 1.0f);  // Blue
  glVertex2f(0.1f, -0.6f);
  glVertex2f(0.7f, -0.6f);
  glVertex2f(0.4f, -0.1f);

  glColor3f(1.0f, 0.0f, 0.0f);  // Red
  glVertex2f(0.3f, -0.4f);
  glColor3f(0.0f, 1.0f, 0.0f);  // Green
  glVertex2f(0.9f, -0.4f);
  glColor3f(0.0f, 0.0f, 1.0f);  // Blue
  glVertex2f(0.6f, -0.9f);
  glEnd();

  glBegin(GL_POLYGON);          // These vertices form a closed polygon
  glColor3f(1.0f, 1.0f, 0.0f);  // Yellow
  glVertex2f(0.4f, 0.2f);
  glVertex2f(0.6f, 0.2f);
  glVertex2f(0.7f, 0.4f);
  glVertex2f(0.6f, 0.6f);
  glVertex2f(0.4f, 0.6f);
  glVertex2f(0.3f, 0.4f);
  glEnd();

  glFlush();  // Render now
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