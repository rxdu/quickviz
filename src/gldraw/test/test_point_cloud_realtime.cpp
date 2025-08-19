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

#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"

#include "core/buffer/buffer_registry.hpp"
#include "core/buffer/ring_buffer.hpp"

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

// Function to generate point cloud data in a separate thread
void GeneratePointCloud(std::string buffer_name, std::atomic<bool>& running) {
  auto& buffer_registry = BufferRegistry::GetInstance();
  auto point_buffer = buffer_registry.GetBuffer<std::vector<glm::vec4>>(buffer_name);
  
  LidarSimulator lidar_sim(10000, 5.0f);
  auto start_time = std::chrono::high_resolution_clock::now();

  while (running) {
    // Get current time in seconds
    auto now = std::chrono::high_resolution_clock::now();
    float time = std::chrono::duration<float>(now - start_time).count();
    
    // Generate new frame of LiDAR data
    auto points = lidar_sim.GenerateFrame(time);
    
    // Write to buffer
    point_buffer->Write(std::move(points));
    
    // Sleep to maintain reasonable frame rate
    std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30 FPS
  }
}

int main(int argc, char* argv[]) {
  // Set up buffer first
  std::string buffer_name = "point_cloud_buffer";
  auto& buffer_registry = BufferRegistry::GetInstance();
  std::shared_ptr<BufferInterface<std::vector<glm::vec4>>> point_buffer =
      std::make_shared<RingBuffer<std::vector<glm::vec4>, 8>>();
  buffer_registry.AddBuffer(buffer_name, point_buffer);

  // Create viewer and scene objects
  Viewer viewer;

  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // Create scene manager
  auto gl_sm = std::make_shared<GlSceneManager>("OpenGL Scene");
  gl_sm->SetAutoLayout(true);
  gl_sm->SetNoTitleBar(true);
  gl_sm->SetFlexGrow(1.0f);
  gl_sm->SetFlexShrink(0.0f);

  // Create point cloud with preallocated buffers
  auto point_cloud = std::make_unique<PointCloud>();
  point_cloud->PreallocateBuffers(10000);
  point_cloud->SetPointSize(3.0f);
  point_cloud->SetOpacity(1.0f);
  point_cloud->SetScalarRange(0.0f, 1.0f);
  point_cloud->SetRenderMode(PointMode::kPoint);
  point_cloud->SetBufferUpdateStrategy(PointCloud::BufferUpdateStrategy::kAuto);
  point_cloud->SetBufferUpdateThreshold(5000);

  // Add a grid for reference
  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  
  // Get pointer to point cloud before moving it
  auto* point_cloud_ptr = point_cloud.get();
  
  // Add objects to scene manager
  gl_sm->AddOpenGLObject("point_cloud", std::move(point_cloud));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // Set up pre-draw callback to update point cloud from buffer
  gl_sm->SetPreDrawCallback([point_cloud_ptr, buffer_name]() {
    auto& buffer_registry = BufferRegistry::GetInstance();
    auto point_buffer = buffer_registry.GetBuffer<std::vector<glm::vec4>>(buffer_name);
    
    std::vector<glm::vec4> points;
    if (point_buffer->Read(points)) {
      point_cloud_ptr->SetPoints(std::move(points), PointCloud::ColorMode::kScalarField);
    }
  });

  // Add scene manager to box and add it to the viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  // Start point cloud generation thread
  std::atomic<bool> running{true};
  std::thread generate_thread(GeneratePointCloud, buffer_name, std::ref(running));

  // Show the viewer
  viewer.Show();
  
  // Clean up
  running = false;
  if (generate_thread.joinable()) {
    generate_thread.join();
  }

  return 0;
} 