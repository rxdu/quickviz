/**
 * @file test_sphere_selection.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Test for sphere object selection functionality
 *
 * This test focuses specifically on sphere selection, demonstrating:
 * - Individual sphere selection with visual feedback
 * - Multi-selection with different interaction modes
 * - Sphere highlighting and bounding box computation
 * - Large sphere grids for performance testing
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "selection_test_utils.hpp"
#include "gldraw/renderable/sphere.hpp"
#include <random>

using namespace quickviz;
using namespace selection_test_utils;

/**
 * @brief Test application for sphere selection functionality
 */
class SphereSelectionTest : public SelectionTestApp {
 public:
  SphereSelectionTest() : SelectionTestApp("Sphere Selection Test") {}

  void SetupTestObjects(GlSceneManager* scene_manager) override {
    // Create different sphere configurations for testing
    SetupBasicSphereGrid(scene_manager);
    SetupLayeredSpheres(scene_manager);
    SetupPerformanceTestSpheres(scene_manager);
  }

  std::string GetTestDescription() const override {
    return "Interactive test for sphere selection functionality.\n"
           "Tests individual selection, multi-selection, and visual feedback\n"
           "for sphere objects in various configurations.";
  }

  std::string GetInstructions() const override {
    return "=== Mouse Controls ===\n"
           "- Left Click: Select sphere\n"
           "- Ctrl+Shift+Click: Add sphere to selection\n"
           "- Ctrl+Alt+Click: Toggle sphere selection\n"
           "- Ctrl+Right Click: Clear all selections\n"
           "\n"
           "=== Keyboard Shortcuts ===\n"
           "- O: Object selection mode (spheres only)\n"
           "- H: Hybrid selection mode (default)\n"
           "- C: Clear selection\n"
           "\n"
           "=== Test Features ===\n"
           "- Individual sphere highlighting (yellow glow)\n"
           "- Multi-selection support\n"
           "- Performance with 50+ spheres\n"
           "- Different sphere sizes and colors";
  }

 private:
  void SetupBasicSphereGrid(GlSceneManager* scene_manager) {
    // Create a 3x3 grid of spheres at ground level
    auto positions = TestHelpers::GenerateGridPositions(
        glm::ivec3(3, 3, 1), 4.0f, glm::vec3(0, 0, 1));
    
    TestObjectFactory::CreateSphereGrid(scene_manager, positions, "basic_sphere", 1.0f);
    
    std::cout << "✓ Created basic sphere grid: 3x3 = " << positions.size() << " spheres" << std::endl;
  }

  void SetupLayeredSpheres(GlSceneManager* scene_manager) {
    // Create smaller spheres in multiple layers
    std::vector<glm::vec3> layer_positions;
    
    // Layer 1: Small spheres at z=3
    for (int i = 0; i < 5; ++i) {
      float angle = i * 72.0f * M_PI / 180.0f; // Pentagon
      float radius = 8.0f;
      layer_positions.emplace_back(
          radius * cos(angle), radius * sin(angle), 3.0f);
    }
    
    // Layer 2: Tiny spheres at z=5
    for (int i = 0; i < 7; ++i) {
      float angle = i * 51.43f * M_PI / 180.0f; // Heptagon
      float radius = 6.0f;
      layer_positions.emplace_back(
          radius * cos(angle), radius * sin(angle), 5.0f);
    }
    
    // Create spheres with different sizes
    for (size_t i = 0; i < layer_positions.size(); ++i) {
      auto sphere = std::make_unique<Sphere>(
          layer_positions[i], (i < 5) ? 0.8f : 0.5f);
      
      // Different colors for layers
      glm::vec3 color = (i < 5) ? 
          glm::vec3(1.0f, 0.6f, 0.2f) :  // Orange for first layer
          glm::vec3(0.6f, 0.2f, 1.0f);   // Purple for second layer
      
      sphere->SetColor(color);
      sphere->SetRenderMode(Sphere::RenderMode::kSolid);

      char name_buffer[64];
      snprintf(name_buffer, sizeof(name_buffer), "layer_sphere_%02zu", i);
      scene_manager->AddOpenGLObject(std::string(name_buffer), std::move(sphere));
    }
    
    std::cout << "✓ Created layered spheres: " << layer_positions.size() << " spheres in 2 layers" << std::endl;
  }

  void SetupPerformanceTestSpheres(GlSceneManager* scene_manager) {
    // Create many small spheres for performance testing
    std::vector<glm::vec3> perf_positions;
    
    // Random distribution of small spheres
    std::mt19937 rng(42); // Fixed seed for reproducibility
    std::uniform_real_distribution<float> pos_dist(-15.0f, 15.0f);
    std::uniform_real_distribution<float> height_dist(6.0f, 10.0f);
    
    const size_t num_perf_spheres = 30;
    for (size_t i = 0; i < num_perf_spheres; ++i) {
      perf_positions.emplace_back(
          pos_dist(rng), pos_dist(rng), height_dist(rng));
    }
    
    // Create small spheres with random colors
    for (size_t i = 0; i < perf_positions.size(); ++i) {
      auto sphere = std::make_unique<Sphere>(perf_positions[i], 0.3f);
      
      // Random bright colors
      std::uniform_real_distribution<float> color_dist(0.3f, 1.0f);
      glm::vec3 color(color_dist(rng), color_dist(rng), color_dist(rng));
      sphere->SetColor(color);
      sphere->SetRenderMode(Sphere::RenderMode::kSolid);

      char name_buffer[64];
      snprintf(name_buffer, sizeof(name_buffer), "perf_sphere_%02zu", i);
      scene_manager->AddOpenGLObject(std::string(name_buffer), std::move(sphere));
    }
    
    std::cout << "✓ Created performance test spheres: " << perf_positions.size() << " small spheres" << std::endl;
  }
};

int main() {
  SphereSelectionTest app;
  return app.Run();
}