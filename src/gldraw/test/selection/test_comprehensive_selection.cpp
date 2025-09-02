/**
 * @file test_comprehensive_selection.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Comprehensive selection test for all supported renderable types
 *
 * This test combines all supported selection types in one scene:
 * - Spheres (object selection)
 * - Point clouds (individual point selection)
 * - LineStrips (newly implemented selection)
 * - Future: Meshes, Cylinders, etc.
 *
 * Demonstrates mixed-mode selection and overall system integration.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "selection_test_utils.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/line_strip.hpp"
#include "gldraw/feedback/visual_feedback_system.hpp"
#include "core/event/input_event.hpp"
#include <random>
#include <cmath>
#include <unordered_map>
#include <algorithm>

using namespace quickviz;
using namespace selection_test_utils;

/**
 * @brief Comprehensive test for all selection types with enhanced interactions
 */
class ComprehensiveSelectionTest : public SelectionTestApp {
 public:
  ComprehensiveSelectionTest() : SelectionTestApp("Comprehensive Selection Test") {}

  void SetupTestObjects(SceneManager* scene_manager) override {
    SetupMixedObjectScene(scene_manager);
    PrintSceneDescription();
  }

  std::string GetTestDescription() const override {
    return "Comprehensive test for all supported selection types.\n"
           "Combines spheres, point clouds, and LineStrips in one scene\n"
           "to test mixed-mode selection and system integration.";
  }

  std::string GetInstructions() const override {
    return "=== Mouse Controls ===\n"
           "- Left Click: Select any object/point\n"
           "- Ctrl+Shift+Click: Add to multi-selection\n"
           "- Ctrl+Alt+Click: Toggle selection\n"
           "- Ctrl+Right Click: Clear all selections\n"
           "\n"
           "=== Selection Modes (Keyboard) ===\n"
           "- P: Points only (point clouds)\n"
           "- O: Objects only (spheres, lines)\n"
           "- H: Hybrid mode (everything) [DEFAULT]\n"
           "- C: Clear selection\n"
           "\n"
           "=== Visual Feedback ===\n"
           "- Spheres: Yellow highlight with original size\n"
           "- Points: Yellow highlight with increased size\n"
           "- LineStrips: Yellow color with 2x line width\n"
           "\n"
           "=== Test Integration ===\n"
           "- Mixed object types in one scene\n"
           "- Multi-selection across types\n"
           "- Selection priority handling\n"
           "- Performance with diverse objects\n"
           "\n"
           "=== Enhanced Interactions ===\n"
           "- SPACE: Print detailed selection info\n"
           "- R: Randomize colors of selected objects\n"
           "- B: Make selected objects bigger\n"
           "- S: Make selected objects smaller\n"
           "- F: Focus camera on selection\n"
           "- T: Toggle animation of selected objects\n"
           "- I: Show/hide object info overlays";
  }

 private:
  void SetupMixedObjectScene(SceneManager* scene_manager) {
    // Create a realistic mixed scene with all supported selection types
    
    SetupSceneSpheres(scene_manager);
    SetupScenePointClouds(scene_manager);
    SetupSceneLineStrips(scene_manager);
    
    // Add connecting elements to show relationships
    SetupConnectingElements(scene_manager);
  }

  void SetupSceneSpheres(SceneManager* scene_manager) {
    // Create spheres representing key locations/nodes
    std::vector<std::pair<glm::vec3, std::string>> key_locations = {
        {glm::vec3(-8.0f, -8.0f, 2.0f), "start_node"},
        {glm::vec3(0.0f, -8.0f, 2.0f), "checkpoint_a"},
        {glm::vec3(8.0f, -8.0f, 2.0f), "checkpoint_b"},
        {glm::vec3(8.0f, 0.0f, 2.0f), "junction"},
        {glm::vec3(8.0f, 8.0f, 2.0f), "goal_node"},
        {glm::vec3(-8.0f, 8.0f, 2.0f), "observation_post"},
        {glm::vec3(0.0f, 0.0f, 6.0f), "elevated_marker"}
    };
    
    std::vector<glm::vec3> colors = {
        glm::vec3(0.9f, 0.2f, 0.2f),  // Red start
        glm::vec3(0.9f, 0.7f, 0.2f),  // Orange checkpoint
        glm::vec3(0.9f, 0.7f, 0.2f),  // Orange checkpoint
        glm::vec3(0.2f, 0.2f, 0.9f),  // Blue junction
        glm::vec3(0.2f, 0.9f, 0.2f),  // Green goal
        glm::vec3(0.7f, 0.2f, 0.9f),  // Purple observation
        glm::vec3(0.9f, 0.9f, 0.2f)   // Yellow elevated
    };
    
    for (size_t i = 0; i < key_locations.size(); ++i) {
      auto sphere = std::make_unique<Sphere>(key_locations[i].first, 1.2f);
      sphere->SetColor(colors[i]);
      sphere->SetRenderMode(Sphere::RenderMode::kSolid);
      scene_manager->AddOpenGLObject(key_locations[i].second, std::move(sphere));
    }
    
    std::cout << "✓ Created scene spheres: " << key_locations.size() << " key locations" << std::endl;
  }

  void SetupScenePointClouds(SceneManager* scene_manager) {
    // Sensor data visualization as point clouds
    
    // 1. LIDAR scan pattern
    std::vector<glm::vec3> lidar_points;
    std::vector<glm::vec3> lidar_colors;
    
    const int lidar_rays = 180;
    const float max_range = 12.0f;
    const glm::vec3 lidar_origin(-8.0f, -8.0f, 2.5f);  // Near start_node
    
    std::mt19937 rng(789);
    std::uniform_real_distribution<float> range_noise(0.8f, 1.0f);
    
    for (int i = 0; i < lidar_rays; ++i) {
      float angle = (static_cast<float>(i) / lidar_rays) * 3.14159f;  // 180 degrees
      float range = max_range * range_noise(rng);
      
      glm::vec3 point = lidar_origin + glm::vec3(
          range * cos(angle),
          range * sin(angle),
          0.0f
      );
      lidar_points.push_back(point);
      
      // Color by distance
      float dist_ratio = range / max_range;
      lidar_colors.emplace_back(1.0f - dist_ratio, dist_ratio * 0.8f, 0.3f);
    }
    
    auto lidar_cloud = std::make_unique<PointCloud>();
    lidar_cloud->SetPoints(lidar_points, lidar_colors);
    lidar_cloud->SetPointSize(6.0f);
    scene_manager->AddOpenGLObject("lidar_scan", std::move(lidar_cloud));

    // 2. Dense measurement cluster
    std::vector<glm::vec3> measurement_points;
    std::vector<glm::vec3> measurement_colors;
    
    const glm::vec3 measurement_center(0.0f, 0.0f, 3.0f);
    std::normal_distribution<float> cluster_dist(0.0f, 1.2f);
    
    const int measurement_count = 80;
    for (int i = 0; i < measurement_count; ++i) {
      glm::vec3 offset(
          cluster_dist(rng),
          cluster_dist(rng),
          cluster_dist(rng) * 0.4f
      );
      measurement_points.push_back(measurement_center + offset);
      
      // Cool color palette
      measurement_colors.emplace_back(0.2f, 0.6f + offset.z * 0.3f, 0.9f);
    }
    
    auto measurement_cloud = std::make_unique<PointCloud>();
    measurement_cloud->SetPoints(measurement_points, measurement_colors);
    measurement_cloud->SetPointSize(7.0f);
    scene_manager->AddOpenGLObject("measurements", std::move(measurement_cloud));
    
    std::cout << "✓ Created scene point clouds: LIDAR scan (" << lidar_points.size() 
              << " points), measurements (" << measurement_points.size() << " points)" << std::endl;
  }

  void SetupSceneLineStrips(SceneManager* scene_manager) {
    // Connection paths and boundaries
    
    // 1. Main navigation path
    std::vector<glm::vec3> nav_path = {
        glm::vec3(-8.0f, -8.0f, 1.8f),  // start_node
        glm::vec3(-4.0f, -8.0f, 1.8f),  // intermediate
        glm::vec3(0.0f, -8.0f, 1.8f),   // checkpoint_a
        glm::vec3(4.0f, -8.0f, 1.8f),   // intermediate  
        glm::vec3(8.0f, -8.0f, 1.8f),   // checkpoint_b
        glm::vec3(8.0f, -4.0f, 1.8f),   // turn
        glm::vec3(8.0f, 0.0f, 1.8f),    // junction
        glm::vec3(8.0f, 4.0f, 1.8f),    // final approach
        glm::vec3(8.0f, 8.0f, 1.8f)     // goal_node
    };
    auto main_path = std::make_unique<LineStrip>();
    main_path->SetPoints(nav_path);
    main_path->SetColor(glm::vec3(0.0f, 0.9f, 0.9f));  // Cyan path
    main_path->SetLineWidth(4.0f);
    scene_manager->AddOpenGLObject("main_nav_path", std::move(main_path));

    // 2. Alternative route
    std::vector<glm::vec3> alt_path = {
        glm::vec3(0.0f, -8.0f, 1.8f),   // checkpoint_a
        glm::vec3(0.0f, -4.0f, 1.8f),   // turn north
        glm::vec3(0.0f, 0.0f, 1.8f),    // center
        glm::vec3(4.0f, 4.0f, 1.8f),    // approach goal
        glm::vec3(8.0f, 8.0f, 1.8f)     // goal_node
    };
    auto alternative = std::make_unique<LineStrip>();
    alternative->SetPoints(alt_path);
    alternative->SetColor(glm::vec3(1.0f, 0.6f, 0.0f));  // Orange alternative
    alternative->SetLineWidth(3.0f);
    scene_manager->AddOpenGLObject("alternative_path", std::move(alternative));

    // 3. Security perimeter
    std::vector<glm::vec3> perimeter;
    const int perimeter_points = 32;
    const float perimeter_radius = 15.0f;
    const glm::vec3 perimeter_center(0.0f, 0.0f, 0.5f);
    
    for (int i = 0; i < perimeter_points; ++i) {
      float angle = (static_cast<float>(i) / perimeter_points) * 6.283f;
      // Slightly irregular perimeter
      float radius = perimeter_radius + 2.0f * sin(angle * 5.0f);
      perimeter.emplace_back(
          perimeter_center.x + radius * cos(angle),
          perimeter_center.y + radius * sin(angle),
          perimeter_center.z
      );
    }
    
    auto perimeter_line = std::make_unique<LineStrip>();
    perimeter_line->SetPoints(perimeter);
    perimeter_line->SetColor(glm::vec3(1.0f, 0.2f, 0.2f));  // Red boundary
    perimeter_line->SetLineWidth(2.5f);
    perimeter_line->SetClosed(true);
    scene_manager->AddOpenGLObject("security_perimeter", std::move(perimeter_line));
    
    std::cout << "✓ Created scene line strips: main path, alternative, perimeter" << std::endl;
  }

  void SetupConnectingElements(SceneManager* scene_manager) {
    // Visual connection between elevated marker and junction
    std::vector<glm::vec3> connection_line = {
        glm::vec3(0.0f, 0.0f, 6.0f),    // elevated_marker
        glm::vec3(4.0f, 0.0f, 4.0f),    // intermediate
        glm::vec3(8.0f, 0.0f, 2.0f)     // junction
    };
    auto connection = std::make_unique<LineStrip>();
    connection->SetPoints(connection_line);
    connection->SetColor(glm::vec3(0.8f, 0.8f, 0.8f));  // Gray connection
    connection->SetLineWidth(1.5f);
    scene_manager->AddOpenGLObject("connection_line", std::move(connection));
    
    std::cout << "✓ Created connecting elements" << std::endl;
  }

  void PrintSceneDescription() {
    std::cout << "\n=== Scene Description ===" << std::endl;
    std::cout << "Simulated robotics navigation scenario with:" << std::endl;
    std::cout << "\nSPHERES (Key Locations):" << std::endl;
    std::cout << "  - Red: Start Node (-8, -8, 2)" << std::endl;
    std::cout << "  - Orange: Checkpoints (0, -8, 2) and (8, -8, 2)" << std::endl;
    std::cout << "  - Blue: Junction (8, 0, 2)" << std::endl;
    std::cout << "  - Green: Goal Node (8, 8, 2)" << std::endl;
    std::cout << "  - Purple: Observation Post (-8, 8, 2)" << std::endl;
    std::cout << "  - Yellow: Elevated Marker (0, 0, 6)" << std::endl;
    
    std::cout << "\nPOINT CLOUDS (Sensor Data):" << std::endl;
    std::cout << "  - LIDAR scan: 180-degree scan from start position" << std::endl;
    std::cout << "  - Measurements: Dense cluster around center point" << std::endl;
    
    std::cout << "\nLINE STRIPS (Paths & Boundaries):" << std::endl;
    std::cout << "  - Cyan: Main navigation path (start -> goal)" << std::endl;
    std::cout << "  - Orange: Alternative route via center" << std::endl;
    std::cout << "  - Red: Security perimeter (closed boundary)" << std::endl;
    std::cout << "  - Gray: Connection line (elevated -> junction)" << std::endl;
    
    std::cout << "\nTEST FEATURES:" << std::endl;
    std::cout << "  - Mixed selection types in realistic scenario" << std::endl;
    std::cout << "  - Multi-selection across object types" << std::endl;
    std::cout << "  - Selection mode filtering (P/O/H keys)" << std::endl;
    std::cout << "  - Visual feedback for all selection types" << std::endl;
    std::cout << std::endl;
  }

  // Test uses the base class visual feedback system - no custom highlighting needed
};

int main() {
  ComprehensiveSelectionTest app;
  return app.Run();
}
