/*
 * test_cv_image_widget.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>

#include "imview/viewer.hpp"
#include "imview/widget/gl_widget.hpp"
#include "imview/component/opengl/triangle.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  Viewer viewer;

  auto gl_widget = std::make_shared<GlWidget>("OpenGL Scene");
  gl_widget->OnResize(300, 200);
  gl_widget->SetPosition(0, 0);
  gl_widget->UpdateView(glm::mat4(1.0f), glm::mat4(1.0f));
  auto triangle = std::make_unique<Triangle>(1.0f, glm::vec3(0.0f, 0.5f, 0.5f));
  gl_widget->AddOpenGLObject("triangle", std::move(triangle));
  viewer.AddSceneObject(gl_widget);

  viewer.Show();

  return 0;
}