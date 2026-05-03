/**
 * @file test_id_buffer_debug.cpp
 * @author Claude Code
 * @date 2025-01-20
 * @brief Debug test for ID buffer rendering issues
 *
 * This test programmatically triggers selection to debug why only grid cloud
 * IDs are being rendered to the ID buffer while other point clouds are not.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "selection_test_utils.hpp"
#include "scene/renderable/point_cloud.hpp"
#include <iostream>
#include <vector>

using namespace quickviz;
using namespace selection_test_utils;

/**
 * @brief Debug test application for ID buffer rendering
 */
class IdBufferDebugTest : public SelectionTestApp {
 public:
  IdBufferDebugTest() : SelectionTestApp("ID Buffer Debug Test") {}

  void SetupTestObjects(SceneManager* scene_manager) override {
    // Create simple test point clouds at different positions
    SetupSimpleGridPointCloud(scene_manager);
    SetupSimpleSpiralPointCloud(scene_manager);
    SetupSimpleClusterPointCloud(scene_manager);
    
    std::cout << "\n=== Scene Setup Complete ===" << std::endl;
    std::cout << "Now try clicking on different point clouds to see debug output." << std::endl;
    std::cout << "Look for 'Rendering to ID buffer' messages and selection results." << std::endl;
  }

  std::string GetTestDescription() const override {
    return "Debug test for ID buffer rendering issues.\n"
           "Tests programmatic selection to debug why only grid cloud\n"
           "IDs are being rendered to the ID buffer.";
  }

  std::string GetInstructions() const override {
    return "=== Automated Test ===\n"
           "- Test will run automatically\n"
           "- Check console output for debug information\n"
           "- Press ESC to exit\n"
           "\n"
           "=== Debug Information ===\n"
           "- ID buffer rendering debug messages\n"
           "- Point cloud registration info\n"
           "- Selection results";
  }

 private:
  void SetupSimpleGridPointCloud(SceneManager* scene_manager) {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;
    
    // Simple 3x3 grid at Z=1.0
    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 3; ++x) {
        float px = (x - 1) * 2.0f;  // Spacing of 2.0
        float py = (y - 1) * 2.0f;
        points.emplace_back(px, py, 1.0f);
        colors.emplace_back(0.1f, 0.9f, 0.1f);  // Green
      }
    }

    auto grid_cloud = std::make_unique<PointCloud>();
    grid_cloud->SetPoints(points, colors);
    grid_cloud->SetPointSize(15.0f);  // Large for visibility
    scene_manager->AddOpenGLObject("debug_grid", std::move(grid_cloud));
    
    std::cout << "✓ Created debug grid point cloud: " << points.size() << " points at Z=1.0" << std::endl;
  }

  void SetupSimpleSpiralPointCloud(SceneManager* scene_manager) {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;
    
    // Simple spiral at different location and Z level
    for (int i = 0; i < 5; ++i) {
      float angle = i * 1.0f;
      float radius = 1.0f + i * 0.5f;
      points.emplace_back(
          8.0f + radius * cos(angle),  // Offset in X
          0.0f + radius * sin(angle),
          3.0f  // Different Z level
      );
      colors.emplace_back(0.9f, 0.1f, 0.1f);  // Red
    }

    auto spiral_cloud = std::make_unique<PointCloud>();
    spiral_cloud->SetPoints(points, colors);
    spiral_cloud->SetPointSize(15.0f);
    scene_manager->AddOpenGLObject("debug_spiral", std::move(spiral_cloud));
    
    std::cout << "✓ Created debug spiral point cloud: " << points.size() << " points at Z=3.0" << std::endl;
  }
  
  void SetupSimpleClusterPointCloud(SceneManager* scene_manager) {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;
    
    // Simple cluster at another location
    for (int i = 0; i < 4; ++i) {
      points.emplace_back(
          -6.0f + i * 0.8f,  // Line of points
          -6.0f,
          2.0f  // Different Z level
      );
      colors.emplace_back(0.1f, 0.1f, 0.9f);  // Blue
    }

    auto cluster_cloud = std::make_unique<PointCloud>();
    cluster_cloud->SetPoints(points, colors);
    cluster_cloud->SetPointSize(15.0f);
    scene_manager->AddOpenGLObject("debug_cluster", std::move(cluster_cloud));
    
    std::cout << "✓ Created debug cluster point cloud: " << points.size() << " points at Z=2.0" << std::endl;
  }
};

int main() {
  IdBufferDebugTest app;
  return app.Run();
}