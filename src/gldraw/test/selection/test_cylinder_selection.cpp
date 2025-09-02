/**
 * @file test_cylinder_selection.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Test for Cylinder object selection functionality
 *
 * This test validates the Cylinder selection support using the unified
 * GeometricPrimitive interface:
 * - Cylinder highlighting (material-based with highlight colors)
 * - Bounding box calculation for cylinder geometry
 * - Multi-selection with different cylinder configurations
 * - Inherited ID rendering support for GPU picking
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "selection_test_utils.hpp"
#include "gldraw/renderable/cylinder.hpp"
#include <random>
#include <cmath>

using namespace quickviz;
using namespace selection_test_utils;

/**
 * @brief Test application for Cylinder selection functionality
 */
class CylinderSelectionTest : public SelectionTestApp {
 public:
  CylinderSelectionTest() : SelectionTestApp("Cylinder Selection Test") {}

  void SetupTestObjects(SceneManager* scene_manager) override {
    SetupBasicCylinders(scene_manager);
    SetupGeometricVariations(scene_manager);
    SetupTransparentCylinders(scene_manager);
    SetupComplexArrangements(scene_manager);
  }

  std::string GetTestDescription() const override {
    return "Interactive test for Cylinder selection functionality.\n"
           "Tests the unified GeometricPrimitive selection interface with\n"
           "material-based highlighting, bounding box calculation, and ID rendering.";
  }

  std::string GetInstructions() const override {
    return "=== Mouse Controls ===\n"
           "- Left Click: Select Cylinder (highlight color change)\n"
           "- Ctrl+Shift+Click: Add Cylinder to selection\n"
           "- Ctrl+Alt+Click: Toggle Cylinder selection\n"
           "- Ctrl+Right Click: Clear all selections\n"
           "\n"
           "=== Keyboard Shortcuts ===\n"
           "- O: Object selection mode (cylinders only)\n"
           "- H: Hybrid selection mode (default)\n"
           "- C: Clear selection\n"
           "\n"
           "=== Test Features ===\n"
           "- Material-based highlighting system\n"
           "- Accurate bounding box from cylinder geometry\n"
           "- Various cylinder sizes and orientations\n"
           "- Transparency and wireframe mode support\n"
           "- Inherited GPU ID rendering for precise picking";
  }

 private:
  void SetupBasicCylinders(SceneManager* scene_manager) {
    // Standard upright cylinder
    auto standard_cylinder = std::make_unique<Cylinder>(
        glm::vec3(0.0f, 0.0f, 0.0f), 2.0f, 1.0f);
    standard_cylinder->SetColor(glm::vec3(0.8f, 0.3f, 0.3f));  // Red
    scene_manager->AddOpenGLObject("standard_cylinder", std::move(standard_cylinder));

    // Wide short cylinder
    auto wide_cylinder = std::make_unique<Cylinder>(
        glm::vec3(-4.0f, 0.0f, 0.0f), 1.0f, 1.8f);
    wide_cylinder->SetColor(glm::vec3(0.3f, 0.8f, 0.3f));  // Green
    scene_manager->AddOpenGLObject("wide_cylinder", std::move(wide_cylinder));

    // Thin tall cylinder
    auto tall_cylinder = std::make_unique<Cylinder>(
        glm::vec3(4.0f, 0.0f, 0.0f), 4.0f, 0.6f);
    tall_cylinder->SetColor(glm::vec3(0.3f, 0.3f, 0.8f));  // Blue
    scene_manager->AddOpenGLObject("tall_cylinder", std::move(tall_cylinder));

    std::cout << "✓ Created basic cylinders: standard, wide, tall" << std::endl;
  }

  void SetupGeometricVariations(SceneManager* scene_manager) {
    // Tilted cylinder using base and top centers
    auto tilted_cylinder = std::make_unique<Cylinder>();
    tilted_cylinder->SetBaseCenter(glm::vec3(-2.0f, -6.0f, 0.0f));
    tilted_cylinder->SetTopCenter(glm::vec3(2.0f, -4.0f, 2.0f));
    tilted_cylinder->SetRadius(0.8f);
    tilted_cylinder->SetColor(glm::vec3(0.8f, 0.8f, 0.3f));  // Yellow
    scene_manager->AddOpenGLObject("tilted_cylinder", std::move(tilted_cylinder));

    // Horizontal cylinder (lying on side)
    auto horizontal_cylinder = std::make_unique<Cylinder>();
    horizontal_cylinder->SetBaseCenter(glm::vec3(6.0f, -6.0f, 0.0f));
    horizontal_cylinder->SetTopCenter(glm::vec3(10.0f, -6.0f, 0.0f));
    horizontal_cylinder->SetRadius(1.2f);
    horizontal_cylinder->SetColor(glm::vec3(0.8f, 0.3f, 0.8f));  // Magenta
    scene_manager->AddOpenGLObject("horizontal_cylinder", std::move(horizontal_cylinder));

    // Diagonal cylinder
    auto diagonal_cylinder = std::make_unique<Cylinder>();
    diagonal_cylinder->SetBaseCenter(glm::vec3(-8.0f, 2.0f, -1.0f));
    diagonal_cylinder->SetTopCenter(glm::vec3(-6.0f, 4.0f, 3.0f));
    diagonal_cylinder->SetRadius(0.5f);
    diagonal_cylinder->SetColor(glm::vec3(0.3f, 0.8f, 0.8f));  // Cyan
    scene_manager->AddOpenGLObject("diagonal_cylinder", std::move(diagonal_cylinder));

    std::cout << "✓ Created geometric variations: tilted, horizontal, diagonal" << std::endl;
  }

  void SetupTransparentCylinders(SceneManager* scene_manager) {
    // Semi-transparent cylinder
    auto transparent_cylinder = std::make_unique<Cylinder>(
        glm::vec3(0.0f, 6.0f, 2.0f), 1.5f, 1.0f);
    transparent_cylinder->SetColor(glm::vec3(1.0f, 0.5f, 0.0f));  // Orange
    transparent_cylinder->SetRenderMode(Cylinder::RenderMode::kTransparent);
    transparent_cylinder->SetOpacity(0.6f);
    scene_manager->AddOpenGLObject("transparent_cylinder", std::move(transparent_cylinder));

    // Wireframe cylinder
    auto wireframe_cylinder = std::make_unique<Cylinder>(
        glm::vec3(-4.0f, 6.0f, 1.0f), 2.0f, 0.8f);
    wireframe_cylinder->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));  // White
    wireframe_cylinder->SetWireframeColor(glm::vec3(0.9f, 0.9f, 0.9f));
    wireframe_cylinder->SetRenderMode(Cylinder::RenderMode::kWireframe);
    wireframe_cylinder->SetWireframeWidth(2.0f);
    scene_manager->AddOpenGLObject("wireframe_cylinder", std::move(wireframe_cylinder));

    std::cout << "✓ Created transparent cylinders: semi-transparent, wireframe" << std::endl;
  }

  void SetupComplexArrangements(SceneManager* scene_manager) {
    // Array of small cylinders for selection precision testing
    std::mt19937 rng(456);
    std::uniform_real_distribution<float> height_dist(0.5f, 2.0f);
    std::uniform_real_distribution<float> radius_dist(0.3f, 0.8f);
    std::uniform_real_distribution<float> color_dist(0.2f, 0.9f);

    const int grid_size = 4;
    const float spacing = 2.5f;
    const glm::vec3 grid_origin(8.0f, 0.0f, -8.0f);

    for (int i = 0; i < grid_size; ++i) {
      for (int j = 0; j < grid_size; ++j) {
        glm::vec3 position = grid_origin + glm::vec3(i * spacing, 0.0f, j * spacing);
        float height = height_dist(rng);
        float radius = radius_dist(rng);
        
        auto cylinder = std::make_unique<Cylinder>(position, height, radius);
        cylinder->SetColor(glm::vec3(color_dist(rng), color_dist(rng), color_dist(rng)));
        
        std::string name = "grid_cylinder_" + std::to_string(i) + "_" + std::to_string(j);
        scene_manager->AddOpenGLObject(name, std::move(cylinder));
      }
    }

    // Concentric cylinders for layered selection testing
    const int concentric_count = 3;
    const glm::vec3 concentric_center(-10.0f, 0.0f, -6.0f);
    
    for (int i = 0; i < concentric_count; ++i) {
      float height = 1.0f + i * 0.8f;
      float radius = 2.5f - i * 0.6f;
      
      auto cylinder = std::make_unique<Cylinder>(concentric_center, height, radius);
      
      // Different colors and modes for each layer
      if (i == 0) {
        cylinder->SetColor(glm::vec3(0.7f, 0.2f, 0.2f));  // Dark red (outer)
      } else if (i == 1) {
        cylinder->SetColor(glm::vec3(0.2f, 0.7f, 0.2f));  // Dark green (middle)
        cylinder->SetRenderMode(Cylinder::RenderMode::kTransparent);
        cylinder->SetOpacity(0.7f);
      } else {
        cylinder->SetColor(glm::vec3(0.2f, 0.2f, 0.7f));  // Dark blue (inner)
        cylinder->SetRenderMode(Cylinder::RenderMode::kWireframe);
        cylinder->SetWireframeWidth(3.0f);
      }
      
      std::string name = "concentric_cylinder_" + std::to_string(i);
      scene_manager->AddOpenGLObject(name, std::move(cylinder));
    }

    std::cout << "✓ Created complex arrangements: " << (grid_size * grid_size) 
              << " grid cylinders, " << concentric_count << " concentric cylinders" << std::endl;
  }
};

int main() {
  CylinderSelectionTest app;
  return app.Run();
}