/**
 * @file test_point_cloud_selection.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Test for point cloud point selection functionality
 *
 * This test focuses specifically on individual point selection within point clouds:
 * - GPU-based pixel-perfect point picking
 * - Point highlighting through layer system
 * - Multi-point selection patterns
 * - Different point cloud patterns and densities
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "selection_test_utils.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include <random>
#include <cmath>

using namespace quickviz;
using namespace selection_test_utils;

/**
 * @brief Test application for point cloud point selection
 */
class PointCloudSelectionTest : public SelectionTestApp {
 public:
  PointCloudSelectionTest() : SelectionTestApp("Point Cloud Selection Test") {}

  void SetupTestObjects(GlSceneManager* scene_manager) override {
    SetupGridPointCloud(scene_manager);
    SetupSpiralPointCloud(scene_manager);
    SetupDenseClusterPointCloud(scene_manager);
    SetupColorGradientPointCloud(scene_manager);
  }

  std::string GetTestDescription() const override {
    return "Interactive test for individual point selection within point clouds.\n"
           "Tests pixel-perfect point picking, multi-point selection, and\n"
           "visual feedback through the layer system.";
  }

  std::string GetInstructions() const override {
    return "=== Mouse Controls ===\n"
           "- Left Click: Select individual point\n"
           "- Ctrl+Shift+Click: Add point to multi-selection\n"
           "- Ctrl+Alt+Click: Toggle point selection\n"
           "- Ctrl+Right Click: Clear all selections\n"
           "\n"
           "=== Keyboard Shortcuts ===\n"
           "- P: Point selection mode (points only)\n"
           "- H: Hybrid selection mode (default)\n"
           "- C: Clear selection\n"
           "\n"
           "=== Test Features ===\n"
           "- GPU-based pixel-perfect point picking\n"
           "- Point highlighting (yellow, larger size)\n"
           "- Multiple point cloud patterns\n"
           "- Dense point cloud performance\n"
           "- Color-coded point identification";
  }

 private:
  void SetupGridPointCloud(GlSceneManager* scene_manager) {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;
    
    // Regular grid pattern for systematic testing
    const int grid_size = 15;
    const float spacing = 0.8f;
    const float z_level = 1.0f;
    
    for (int y = 0; y < grid_size; ++y) {
      for (int x = 0; x < grid_size; ++x) {
        float px = (x - grid_size / 2) * spacing;
        float py = (y - grid_size / 2) * spacing;
        points.emplace_back(px, py, z_level);

        // Checkerboard color pattern for easy identification
        if ((x + y) % 2 == 0) {
          colors.emplace_back(0.1f, 0.9f, 0.1f);  // Bright green
        } else {
          colors.emplace_back(0.1f, 0.7f, 0.9f);  // Bright cyan
        }
      }
    }

    auto grid_cloud = std::make_unique<PointCloud>();
    grid_cloud->SetPoints(points, colors);
    grid_cloud->SetPointSize(10.0f);  // Large for easy selection
    scene_manager->AddOpenGLObject("test_grid_cloud", std::move(grid_cloud));
    
    std::cout << "✓ Created grid point cloud: " << points.size() << " points" << std::endl;
  }

  void SetupSpiralPointCloud(GlSceneManager* scene_manager) {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;
    
    // Expanding spiral for interesting selection patterns
    const int spiral_points = 80;
    const float max_radius = 8.0f;
    const float z_base = 3.0f;
    const float z_range = 4.0f;
    
    for (int i = 0; i < spiral_points; ++i) {
      float t = static_cast<float>(i) / spiral_points;
      float angle = t * 12.566f; // 2 full rotations
      float radius = max_radius * t;
      float height = z_base + z_range * sin(angle * 2.0f);
      
      points.emplace_back(
          radius * cos(angle),
          radius * sin(angle),
          height
      );
      
      // Color based on height for visual depth cues
      float normalized_height = (height - z_base) / z_range * 0.5f + 0.5f;
      colors.emplace_back(
          1.0f - normalized_height,  // Red decreases with height
          normalized_height * 0.8f,  // Green increases with height
          0.5f + normalized_height * 0.5f  // Blue varies
      );
    }

    auto spiral_cloud = std::make_unique<PointCloud>();
    spiral_cloud->SetPoints(points, colors);
    spiral_cloud->SetPointSize(8.0f);
    scene_manager->AddOpenGLObject("test_spiral_cloud", std::move(spiral_cloud));
    
    std::cout << "✓ Created spiral point cloud: " << points.size() << " points" << std::endl;
  }

  void SetupDenseClusterPointCloud(GlSceneManager* scene_manager) {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;
    
    // Dense cluster for performance and precision testing
    std::mt19937 rng(123);
    std::normal_distribution<float> cluster_dist(0.0f, 1.5f);
    std::uniform_real_distribution<float> color_dist(0.2f, 1.0f);
    
    const int cluster_points = 120;
    const glm::vec3 cluster_center(-10.0f, 8.0f, 2.5f);
    
    for (int i = 0; i < cluster_points; ++i) {
      glm::vec3 offset(
          cluster_dist(rng),
          cluster_dist(rng),
          cluster_dist(rng) * 0.5f  // Flatter in Z
      );
      points.push_back(cluster_center + offset);
      
      // Warm color palette for cluster
      colors.emplace_back(
          color_dist(rng),           // Red component
          color_dist(rng) * 0.7f,    // Orange-ish
          0.2f                       // Low blue
      );
    }

    auto cluster_cloud = std::make_unique<PointCloud>();
    cluster_cloud->SetPoints(points, colors);
    cluster_cloud->SetPointSize(6.0f);  // Smaller for dense packing
    scene_manager->AddOpenGLObject("test_cluster_cloud", std::move(cluster_cloud));
    
    std::cout << "✓ Created dense cluster point cloud: " << points.size() << " points" << std::endl;
  }

  void SetupColorGradientPointCloud(GlSceneManager* scene_manager) {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;
    
    // Linear arrangement with color gradient for visual testing
    const int gradient_points = 40;
    const float start_x = 8.0f;
    const float end_x = 18.0f;
    const float y_pos = -12.0f;
    const float z_pos = 1.5f;
    
    for (int i = 0; i < gradient_points; ++i) {
      float t = static_cast<float>(i) / (gradient_points - 1);
      float x = start_x + t * (end_x - start_x);
      
      // Add slight wave pattern
      float wave_offset = sin(t * 6.283f * 3.0f) * 0.8f;
      
      points.emplace_back(x, y_pos + wave_offset, z_pos);
      
      // Smooth color gradient from blue to red
      colors.emplace_back(t, 0.3f, 1.0f - t);
    }

    auto gradient_cloud = std::make_unique<PointCloud>();
    gradient_cloud->SetPoints(points, colors);
    gradient_cloud->SetPointSize(9.0f);
    scene_manager->AddOpenGLObject("test_gradient_cloud", std::move(gradient_cloud));
    
    std::cout << "✓ Created gradient point cloud: " << points.size() << " points" << std::endl;
  }
};

int main() {
  PointCloudSelectionTest app;
  return app.Run();
}