/*
 * test_camera_enhanced.cpp
 *
 * Created on: Dec 2024
 * Description: Test enhanced camera controller with 3D translation support
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"

using namespace quickviz;

int main() {
  std::cout << "Testing Enhanced Camera Controller with 3D Translation" << std::endl;
  
  // Create a simple point cloud for testing
  std::vector<glm::vec4> test_points;
  for (int i = -5; i <= 5; ++i) {
    for (int j = -5; j <= 5; ++j) {
      for (int k = -5; k <= 5; ++k) {
        float intensity = (i + j + k + 15) / 30.0f; // normalized 0-1
        test_points.push_back(glm::vec4(i, j, k, intensity));
      }
    }
  }
  
  std::cout << "Created test point cloud with " << test_points.size() << " points" << std::endl;
  
  // Create viewer and scene
  Viewer viewer;
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  auto gl_sm = std::make_shared<GlSceneManager>("Enhanced Camera Test");
  gl_sm->SetAutoLayout(true);
  gl_sm->SetNoTitleBar(true);
  gl_sm->SetFlexGrow(1.0f);
  gl_sm->SetFlexShrink(0.0f);
  
  // Create point cloud
  auto point_cloud = std::make_unique<PointCloud>();
  point_cloud->SetPoints(test_points, PointCloud::ColorMode::kScalarField);
  point_cloud->SetScalarRange(0.0f, 1.0f);
  point_cloud->SetPointSize(5.0f);
  point_cloud->SetRenderMode(PointMode::kPoint);
  
  // Add grid for reference
  auto grid = std::make_unique<Grid>(20.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  
  gl_sm->AddOpenGLObject("point_cloud", std::move(point_cloud));
  gl_sm->AddOpenGLObject("grid", std::move(grid));
  
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);
  
  std::cout << "\n=== Camera Controls ===" << std::endl;
  std::cout << "Left Mouse: Rotate camera (orbit mode)" << std::endl;
  std::cout << "Middle Mouse: Translate/Pan in 3D space" << std::endl;
  std::cout << "Scroll Wheel: Zoom in/out" << std::endl;
  std::cout << "Right Mouse: Alternative rotation" << std::endl;
  std::cout << "\nThe middle mouse button now supports 3D translation!" << std::endl;
  
  viewer.Show();
  
  return 0;
}