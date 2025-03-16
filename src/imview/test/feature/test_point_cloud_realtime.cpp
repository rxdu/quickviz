/*
 * test_point_cloud_realtime.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description: Interactive point cloud visualization with real-time updates
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <thread>
#include <random>
#include <chrono>
#include <vector>
#include <atomic>

#include "imview/box.hpp"
#include "imview/viewer.hpp"
#include "imview/component/opengl/renderer/grid.hpp"
#include "imview/component/opengl/renderer/point_cloud.hpp"
#include "imview/component/opengl/gl_scene_manager.hpp"

using namespace quickviz;

// Simulated LiDAR data generator
class LidarSimulator {
 public:
  LidarSimulator(size_t max_points, float range)
      : max_points_(max_points), range_(range) {
    // Initialize random number generators
    std::random_device rd;
    gen_ = std::mt19937(rd());
    dist_ = std::uniform_real_distribution<float>(-range_, range_);
  }

  // Generate a frame of LiDAR data
  std::vector<glm::vec4> GenerateFrame(float time) {
    // Number of points varies between frames
    size_t num_points = max_points_ * (0.5f + 0.5f * std::sin(time * 0.5f));
    std::vector<glm::vec4> points;
    points.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
      // Generate points in a rotating pattern
      float angle = time + i * 0.01f;
      float radius = range_ * (0.2f + 0.8f * std::abs(dist_(gen_)) / range_);
      float x = radius * std::cos(angle);
      float y = radius * std::sin(angle);
      float z = dist_(gen_) * 0.2f;  // Less variation in z
      
      // Use distance from center as intensity (w component)
      float intensity = radius / range_;
      
      points.push_back(glm::vec4(x, y, z, intensity));
    }

    return points;
  }

 private:
  size_t max_points_;
  float range_;
  std::mt19937 gen_;
  std::uniform_real_distribution<float> dist_;
};

int main(int argc, char* argv[]) {
  Viewer viewer;

  // Create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // Create a OpenGL scene manager to manage the OpenGL objects
  auto gl_sm = std::make_shared<GlSceneManager>("OpenGL Scene");
  gl_sm->SetAutoLayout(true);
  gl_sm->SetNoTitleBar(true);
  gl_sm->SetFlexGrow(1.0f);
  gl_sm->SetFlexShrink(0.0f);

  // Create LiDAR simulator
  const size_t MAX_POINTS = 10000;
  LidarSimulator lidar_sim(MAX_POINTS, 5.0f);

  // Create point cloud with preallocated buffers
  auto point_cloud = std::make_unique<PointCloud>();
  point_cloud->PreallocateBuffers(MAX_POINTS);
  point_cloud->SetPointSize(3.0f);
  point_cloud->SetOpacity(1.0f);
  point_cloud->SetScalarRange(0.0f, 1.0f);
  point_cloud->SetRenderMode(PointRenderMode::Points);
  
  // Configure buffer update strategy
  point_cloud->SetBufferUpdateStrategy(PointCloud::BufferUpdateStrategy::kAuto);
  point_cloud->SetBufferUpdateThreshold(5000); // Use buffer mapping for more than 5000 points

  // Add a grid for reference
  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  
  // Add objects to scene manager
  gl_sm->AddOpenGLObject("point_cloud", std::move(point_cloud));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // Add scene manager to box and add it to the viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  // Flag to signal when to stop the background thread
  std::atomic<bool> running{true};

  // Start a background thread to generate point cloud data
  std::thread update_thread([&]() {
    auto start_time = std::chrono::high_resolution_clock::now();
    float point_size = 3.0f;
    bool increasing_size = true;
    
    while (running) {
      // Get current time in seconds
      auto now = std::chrono::high_resolution_clock::now();
      float time = std::chrono::duration<float>(now - start_time).count();
      
      // Generate new frame of LiDAR data (this is thread-safe, no OpenGL calls)
      auto points = lidar_sim.GenerateFrame(time);
      
      // Dynamically change point size to demonstrate automatic strategy selection
      if (increasing_size) {
        point_size += 0.1f;
        if (point_size > 10.0f) increasing_size = false;
      } else {
        point_size -= 0.1f;
        if (point_size < 1.0f) increasing_size = true;
      }
      
      // Measure update time
      auto update_start = std::chrono::high_resolution_clock::now();
      
      // Get point cloud from scene manager and update it (thread-safe now)
      auto* point_cloud = static_cast<PointCloud*>(
          gl_sm->GetOpenGLObject("point_cloud"));
      if (point_cloud) {
        point_cloud->SetPointSize(point_size);
        // Use move semantics for better performance
        point_cloud->SetPoints(std::move(points), PointCloud::ColorMode::kScalarField);
      }
      
      auto update_end = std::chrono::high_resolution_clock::now();
      auto update_duration = std::chrono::duration<float, std::milli>(
          update_end - update_start).count();
      
      std::cout << "Updated points with size " << point_size
                << " in " << update_duration << " ms" << std::endl;
      
      // Sleep to maintain reasonable frame rate
      std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30 FPS
    }
  });

  // Show the viewer
  viewer.Show();
  
  // Clean up
  running = false;
  if (update_thread.joinable()) {
    update_thread.join();
  }

  return 0;
} 