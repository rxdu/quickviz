/*
 * test_point_cloud.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description: Interactive point cloud visualization with camera controls
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <thread>
#include <random>

#include "imview/box.hpp"
#include "imview/viewer.hpp"
#include "imview/component/opengl/grid.hpp"
#include "imview/component/opengl/point_cloud.hpp"
#include "imview/component/opengl/gl_scene_manager.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  Viewer viewer;

  // create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // create a OpenGL scene manager to manage the OpenGL objects
  auto gl_sm = std::make_shared<GlSceneManager>("OpenGL Scene");
  gl_sm->SetAutoLayout(true);
  gl_sm->SetNoTitleBar(true);
  gl_sm->SetFlexGrow(1.0f);
  gl_sm->SetFlexShrink(0.0f);

  // now add the rendering objects to the OpenGL scene manager
  std::cout << "Generating random points..." << std::endl;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> dist(
      0.0f, 0.5f);  // Standard deviation of 0.5 instead of 1.0

  // Generate 3D points with a Gaussian distribution
  std::vector<glm::vec3> points;
  std::vector<glm::vec3> colors;

  // Generate 1000 random points
  for (int i = 0; i < 1000; ++i) {
    float x = dist(gen) * 10;
    float y = dist(gen) * 10;
    float z = dist(gen) * 10;
    points.push_back(glm::vec3(x, y, z));

    // Use bright colors for better visibility
    // Map each point to a bright color based on its position
    colors.push_back(glm::vec3(
        fabs(x) + 0.5f,  // Brighter red (0.5-1.5 range)
        fabs(y) + 0.5f,  // Brighter green (0.5-1.5 range)
        fabs(z) + 0.5f   // Brighter blue (0.5-1.5 range)
        ));
  }

  std::cout << "Generated " << points.size()
            << " random points with Gaussian distribution" << std::endl;

  // Add a grid for reference
  // auto grid = std::make_unique<Grid>(10.0f, 1.0f);
  // gl_widget->AddOpenGLObject("grid", std::move(grid));

  // Create and configure point cloud with very large points
  auto point_cloud = std::make_unique<PointCloud>();
  point_cloud->SetPoints(points);
  point_cloud->SetColors(colors);  // Use direct colors
  point_cloud->SetPointSize(
      50.0f);  // Make points extremely large for visibility
  point_cloud->SetOpacity(1.0f);
  point_cloud->SetRenderMode(PointRenderMode::Points);

  // Add this line to print the point cloud data for debugging
  std::cout << "First point: " << points[0].x << ", " << points[0].y << ", "
            << points[0].z << std::endl;
  std::cout << "First color: " << colors[0].x << ", " << colors[0].y << ", "
            << colors[0].z << std::endl;

  gl_sm->AddOpenGLObject("point_cloud", std::move(point_cloud));

  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // finally pass the OpenGL scene manager to the box and add it to the viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}