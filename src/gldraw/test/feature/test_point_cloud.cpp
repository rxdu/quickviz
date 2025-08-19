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

#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"

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
  std::vector<glm::vec4> points;
  std::vector<glm::vec3> colors;

  // Generate 1000 random points
  for (int i = 0; i < 1000; ++i) {
    float x = dist(gen);
    float y = dist(gen);
    float z = dist(gen);
    points.push_back(glm::vec4(x, y, z, 0.0f));  // Using w=0 as default

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

  // Create point clouds with different color modes
  
  // 1. Static color mode
  auto point_cloud_static = std::make_unique<PointCloud>();
  point_cloud_static->SetPoints(points, PointCloud::ColorMode::kStatic);
  point_cloud_static->SetDefaultColor(glm::vec3(0.25f, 0.0f, 1.0f)); // Purple
  point_cloud_static->SetPointSize(3.0f);
  point_cloud_static->SetOpacity(1.0f);
  point_cloud_static->SetRenderMode(PointMode::kPoint);
  
  // 2. Height field color mode
  auto point_cloud_height = std::make_unique<PointCloud>();
  point_cloud_height->SetScalarRange(-0.5f, 0.5f); // Set range for z values
  point_cloud_height->SetPoints(points, PointCloud::ColorMode::kHeightField);
  point_cloud_height->SetPointSize(3.0f);
  point_cloud_height->SetOpacity(1.0f);
  point_cloud_height->SetRenderMode(PointMode::kPoint);
  
  // 3. Scalar field color mode - set w component to distance from origin
  std::vector<glm::vec4> scalar_points = points;
  for (size_t i = 0; i < scalar_points.size(); ++i) {
    // Set w component to distance from origin
    scalar_points[i].w = glm::length(glm::vec3(scalar_points[i]));
  }
  
  auto point_cloud_scalar = std::make_unique<PointCloud>();
  point_cloud_scalar->SetScalarRange(0.0f, 1.0f); // Set range for scalar values
  point_cloud_scalar->SetPoints(scalar_points, PointCloud::ColorMode::kScalarField);
  point_cloud_scalar->SetPointSize(3.0f);
  point_cloud_scalar->SetOpacity(1.0f);
  point_cloud_scalar->SetRenderMode(PointMode::kPoint);

  // Add this line to print the point cloud data for debugging
  std::cout << "First point: " << points[0].x << ", " << points[0].y << ", "
            << points[0].z << ", " << points[0].w << std::endl;

  // Add all point clouds to the scene manager
  // (Uncomment one of these lines to visualize the other color modes)
  // gl_sm->AddOpenGLObject("point_cloud_static", std::move(point_cloud_static));
  gl_sm->AddOpenGLObject("point_cloud_height", std::move(point_cloud_height));
  // gl_sm->AddOpenGLObject("point_cloud_scalar", std::move(point_cloud_scalar));
  
  // Add a grid for reference
  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // finally pass the OpenGL scene manager to the box and add it to the viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}