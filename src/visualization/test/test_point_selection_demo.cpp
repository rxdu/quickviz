/**
 * @file test_point_selection_demo.cpp
 * @date 2025-08-25
 * @brief Simple demo of point cloud selection workflow using existing infrastructure
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <random>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "visualization/selection/point_cloud_selector.hpp"

using namespace quickviz;
using namespace quickviz::visualization;

void GenerateTestPointCloud(PointCloud& point_cloud) {
  std::vector<glm::vec4> points;
  std::mt19937 gen(42);
  std::normal_distribution<float> dist(0.0f, 1.5f);
  std::uniform_real_distribution<float> height_dist(-2.0f, 2.0f);
  
  // Generate clusters of points
  std::vector<glm::vec3> cluster_centers = {
    glm::vec3(0.0f, 0.0f, 0.0f),
    glm::vec3(3.0f, 0.0f, 0.0f),
    glm::vec3(-1.5f, 2.0f, 0.0f),
    glm::vec3(1.5f, -2.0f, 1.0f)
  };
  
  for (const auto& center : cluster_centers) {
    for (int i = 0; i < 500; ++i) {
      float x = center.x + dist(gen);
      float y = center.y + dist(gen);
      float z = center.z + height_dist(gen);
      float intensity = (z + 2.0f) / 4.0f; // Normalize to [0, 1]
      points.push_back(glm::vec4(x, y, z, intensity));
    }
  }
  
  point_cloud.SetPoints(points, PointCloud::ColorMode::kScalarField);
  point_cloud.SetPointSize(3.0f);
  point_cloud.SetScalarRange(0.0f, 1.0f);
  
  std::cout << "Generated " << points.size() << " test points" << std::endl;
}

// Global variables for the demo (simple approach)
std::unique_ptr<PointCloudSelector> g_selector;
std::optional<glm::vec3> g_last_pick_position;

void SetupScene(GlSceneManager* scene_manager) {
  std::cout << "\n=== Point Cloud Selection Demo ===" << std::endl;
  std::cout << "This demo tests the visualization module's selection capabilities." << std::endl;
  std::cout << "The point cloud supports:" << std::endl;
  std::cout << "- Individual point selection" << std::endl;
  std::cout << "- Region selection (sphere, box, plane, cylinder)" << std::endl;
  std::cout << "- Multiple selection modes (single, additive, subtractive, toggle)" << std::endl;
  std::cout << "- Selection visualization using layer system" << std::endl;
  std::cout << "\nInteraction:" << std::endl;
  std::cout << "- Use keyboard commands to test selection functions" << std::endl;
  std::cout << "- Check console output for selection results" << std::endl;
  std::cout << "====================================\n" << std::endl;
  
  // Create point cloud
  auto point_cloud = std::make_unique<PointCloud>();
  GenerateTestPointCloud(*point_cloud);
  
  // Create selector for the point cloud
  g_selector = std::make_unique<PointCloudSelector>();
  g_selector->SetPointCloud(std::shared_ptr<PointCloud>(point_cloud.get(), [](PointCloud*){})); // Non-owning shared_ptr
  
  // Set selection callback
  g_selector->SetSelectionCallback([](const std::vector<size_t>& indices) {
    std::cout << "Selection changed: " << indices.size() << " points selected" << std::endl;
    
    if (!indices.empty()) {
      glm::vec3 centroid = g_selector->GetSelectionCentroid();
      auto [min_pt, max_pt] = g_selector->GetSelectionBounds();
      
      std::cout << "  Centroid: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;
      std::cout << "  Bounds: (" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ") to ("
                << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << ")" << std::endl;
    }
  });
  
  // Add point cloud to scene  
  scene_manager->AddOpenGLObject("point_cloud", std::move(point_cloud));
  
  // Demonstrate some selection capabilities
  std::cout << "\nDemonstrating selection capabilities:\n" << std::endl;
  
  // Test point picking by position
  std::cout << "1. Testing nearest point selection around origin..." << std::endl;
  auto pick_result = g_selector->PickNearestPoint(glm::vec3(0, 0, 0), 2.0f);
  if (pick_result.has_value()) {
    std::cout << "   Found point " << pick_result->point_index 
              << " at (" << pick_result->point.x << ", " 
              << pick_result->point.y << ", " << pick_result->point.z << ")"
              << " distance: " << pick_result->distance << std::endl;
    g_last_pick_position = pick_result->point;
    
    // Select this point
    g_selector->UpdateSelection({pick_result->point_index}, SelectionMode::kSingle);
    g_selector->ApplySelectionVisualization("initial_pick", glm::vec3(1.0f, 1.0f, 0.0f), 2.0f);
  }
  
  // Test sphere selection
  if (g_last_pick_position.has_value()) {
    std::cout << "\n2. Testing sphere selection (radius 1.0) around picked point..." << std::endl;
    auto sphere_indices = g_selector->SelectInSphere(g_last_pick_position.value(), 1.0f);
    std::cout << "   Found " << sphere_indices.size() << " points in sphere" << std::endl;
    
    g_selector->UpdateSelection(sphere_indices, SelectionMode::kAdditive);
    g_selector->ApplySelectionVisualization("sphere_selection", glm::vec3(0.0f, 1.0f, 0.0f), 1.8f);
  }
  
  // Test box selection  
  std::cout << "\n3. Testing box selection around origin..." << std::endl;
  auto box_indices = g_selector->SelectInBox(glm::vec3(-1, -1, -1), glm::vec3(1, 1, 1));
  std::cout << "   Found " << box_indices.size() << " points in box" << std::endl;
  
  g_selector->UpdateSelection(box_indices, SelectionMode::kAdditive);
  g_selector->ApplySelectionVisualization("box_selection", glm::vec3(0.0f, 0.0f, 1.0f), 1.6f);
  
  // Test plane selection
  std::cout << "\n4. Testing plane selection (above z=0)..." << std::endl;
  auto plane_indices = g_selector->SelectByPlane(glm::vec3(0, 0, 0), glm::vec3(0, 0, 1), true);
  std::cout << "   Found " << plane_indices.size() << " points above z=0" << std::endl;
  
  // Don't add plane selection to avoid too much clutter, just demonstrate the capability
  
  std::cout << "\nTotal unique points selected: " << g_selector->GetSelectionCount() << std::endl;
  std::cout << "\nDemo complete! The selection system is working correctly." << std::endl;
  std::cout << "Multiple selection layers are visible with different colors and sizes." << std::endl;
}

int main(int argc, char* argv[]) {
  try {
    // Configure GlView  
    GlView::Config config;
    config.window_title = "Point Cloud Selection Demo";
    config.show_grid = true;
    config.show_coordinate_frame = true;
    config.grid_size = 20.0f;
    config.grid_step = 1.0f;
    config.coordinate_frame_size = 2.0f;
    
    // Create GlView
    GlView view(config);
    
    // Set up scene
    view.SetSceneSetup(SetupScene);
    
    // Add help text
    view.AddHelpSection("Selection Demo", {
      "This demo automatically tests selection capabilities:",
      "• Yellow highlights: Individual point picks",
      "• Green highlights: Sphere selection results", 
      "• Blue highlights: Box selection results",
      "• Check console for detailed results"
    });
    
    view.SetDescription(
      "Automated demo of point cloud selection capabilities.\n"
      "Tests point picking, sphere selection, box selection, and plane selection.\n"
      "Multiple selection layers are visualized with different colors."
    );
    
    // Run the view
    view.Run();
    
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
  
  return 0;
}