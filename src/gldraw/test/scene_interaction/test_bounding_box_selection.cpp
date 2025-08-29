/**
 * @file test_bounding_box_selection.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Test for BoundingBox object selection functionality
 *
 * This test validates the BoundingBox selection support using the unified
 * GeometricPrimitive interface:
 * - BoundingBox highlighting (material-based with highlight colors)
 * - Accurate bounding box calculation for selection
 * - Multi-selection with different box configurations
 * - Inherited ID rendering support for GPU picking
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "selection_test_utils.hpp"
#include "gldraw/renderable/bounding_box.hpp"
#include <random>
#include <cmath>

using namespace quickviz;
using namespace selection_test_utils;

/**
 * @brief Test application for BoundingBox selection functionality
 */
class BoundingBoxSelectionTest : public SelectionTestApp {
 public:
  BoundingBoxSelectionTest() : SelectionTestApp("BoundingBox Selection Test") {}

  void SetupTestObjects(GlSceneManager* scene_manager) override {
    SetupBasicBoundingBoxes(scene_manager);
    SetupGeometricVariations(scene_manager);
    SetupTransformBoundingBoxes(scene_manager);
    SetupComplexArrangements(scene_manager);
  }

  std::string GetTestDescription() const override {
    return "Interactive test for BoundingBox selection functionality.\n"
           "Tests the unified GeometricPrimitive selection interface with\n"
           "material-based highlighting, bounding box calculation, and ID rendering.";
  }

  std::string GetInstructions() const override {
    return "=== Mouse Controls ===\n"
           "- Left Click: Select BoundingBox (highlight color change)\n"
           "- Ctrl+Shift+Click: Add BoundingBox to selection\n"
           "- Ctrl+Alt+Click: Toggle BoundingBox selection\n"
           "- Ctrl+Right Click: Clear all selections\n"
           "\n"
           "=== Keyboard Shortcuts ===\n"
           "- O: Object selection mode (bounding boxes only)\n"
           "- H: Hybrid selection mode (default)\n"
           "- C: Clear selection\n"
           "\n"
           "=== Test Features ===\n"
           "- Material-based highlighting system\n"
           "- Accurate bounding box from local geometry\n"
           "- Various box sizes and orientations\n"
           "- Solid and wireframe rendering modes\n"
           "- Transform matrix support for positioning\n"
           "- Inherited GPU ID rendering for precise picking";
  }

 private:
  void SetupBasicBoundingBoxes(GlSceneManager* scene_manager) {
    // Standard axis-aligned bounding box
    auto standard_box = std::make_unique<BoundingBox>(
        glm::vec3(-1.0f, -1.0f, -1.0f), glm::vec3(1.0f, 1.0f, 1.0f));
    standard_box->SetColor(glm::vec3(0.8f, 0.3f, 0.3f));  // Red
    standard_box->SetEdgeColor(glm::vec3(0.5f, 0.1f, 0.1f));
    scene_manager->AddOpenGLObject("standard_box", std::move(standard_box));

    // Wide flat box
    auto wide_box = std::make_unique<BoundingBox>(
        glm::vec3(-6.0f, -0.2f, -1.0f), glm::vec3(-3.0f, 0.2f, 1.0f));
    wide_box->SetColor(glm::vec3(0.3f, 0.8f, 0.3f));  // Green
    wide_box->SetEdgeColor(glm::vec3(0.1f, 0.5f, 0.1f));
    scene_manager->AddOpenGLObject("wide_box", std::move(wide_box));

    // Tall narrow box
    auto tall_box = std::make_unique<BoundingBox>(
        glm::vec3(3.5f, -1.0f, -0.5f), glm::vec3(4.5f, 4.0f, 0.5f));
    tall_box->SetColor(glm::vec3(0.3f, 0.3f, 0.8f));  // Blue
    tall_box->SetEdgeColor(glm::vec3(0.1f, 0.1f, 0.5f));
    scene_manager->AddOpenGLObject("tall_box", std::move(tall_box));

    std::cout << "✓ Created basic bounding boxes: standard, wide, tall" << std::endl;
  }

  void SetupGeometricVariations(GlSceneManager* scene_manager) {
    // Small cube
    auto small_cube = std::make_unique<BoundingBox>(
        glm::vec3(-0.5f, -6.0f, -0.5f), glm::vec3(0.5f, -5.0f, 0.5f));
    small_cube->SetColor(glm::vec3(0.8f, 0.8f, 0.3f));  // Yellow
    small_cube->SetEdgeColor(glm::vec3(0.5f, 0.5f, 0.1f));
    scene_manager->AddOpenGLObject("small_cube", std::move(small_cube));

    // Large box
    auto large_box = std::make_unique<BoundingBox>(
        glm::vec3(6.0f, -2.0f, -2.0f), glm::vec3(10.0f, 2.0f, 2.0f));
    large_box->SetColor(glm::vec3(0.8f, 0.3f, 0.8f));  // Magenta
    large_box->SetEdgeColor(glm::vec3(0.5f, 0.1f, 0.5f));
    scene_manager->AddOpenGLObject("large_box", std::move(large_box));

    // Deep box (extended in Z)
    auto deep_box = std::make_unique<BoundingBox>(
        glm::vec3(-8.0f, 1.0f, -4.0f), glm::vec3(-6.0f, 3.0f, 2.0f));
    deep_box->SetColor(glm::vec3(0.3f, 0.8f, 0.8f));  // Cyan
    deep_box->SetEdgeColor(glm::vec3(0.1f, 0.5f, 0.5f));
    scene_manager->AddOpenGLObject("deep_box", std::move(deep_box));

    std::cout << "✓ Created geometric variations: small cube, large box, deep box" << std::endl;
  }

  void SetupTransformBoundingBoxes(GlSceneManager* scene_manager) {
    // Wireframe box using SetCenter
    auto wireframe_box = std::make_unique<BoundingBox>();
    wireframe_box->SetCenter(glm::vec3(0.0f, 6.0f, 2.0f), glm::vec3(2.0f, 1.0f, 1.5f));
    wireframe_box->SetColor(glm::vec3(1.0f, 0.5f, 0.0f));  // Orange
    wireframe_box->SetEdgeColor(glm::vec3(1.0f, 1.0f, 1.0f)); // White edges
    wireframe_box->SetRenderMode(BoundingBox::RenderMode::kWireframe);
    wireframe_box->SetWireframeWidth(2.0f);
    wireframe_box->SetShowCornerPoints(true, 8.0f);
    scene_manager->AddOpenGLObject("wireframe_box", std::move(wireframe_box));

    // Transparent box
    auto transparent_box = std::make_unique<BoundingBox>(
        glm::vec3(-4.0f, 6.0f, 0.0f), glm::vec3(-2.0f, 8.0f, 1.0f));
    transparent_box->SetColor(glm::vec3(1.0f, 0.0f, 1.0f));  // Bright magenta
    transparent_box->SetRenderMode(BoundingBox::RenderMode::kTransparent);
    transparent_box->SetOpacity(0.4f);
    transparent_box->SetEdgeColor(glm::vec3(0.8f, 0.0f, 0.8f));
    transparent_box->SetShowEdges(true);
    transparent_box->SetEdgeWidth(3.0f);
    scene_manager->AddOpenGLObject("transparent_box", std::move(transparent_box));

    std::cout << "✓ Created transform bounding boxes: wireframe with corners, transparent with edges" << std::endl;
  }

  void SetupComplexArrangements(GlSceneManager* scene_manager) {
    // Array of small boxes for selection precision testing
    std::mt19937 rng(789);
    std::uniform_real_distribution<float> size_dist(0.3f, 1.2f);
    std::uniform_real_distribution<float> color_dist(0.2f, 0.9f);

    const int grid_size = 4;
    const float spacing = 2.5f;
    const glm::vec3 grid_origin(8.0f, 0.0f, -8.0f);

    for (int i = 0; i < grid_size; ++i) {
      for (int j = 0; j < grid_size; ++j) {
        glm::vec3 position = grid_origin + glm::vec3(i * spacing, 0.0f, j * spacing);
        glm::vec3 size = glm::vec3(size_dist(rng), size_dist(rng), size_dist(rng));
        
        auto box = std::make_unique<BoundingBox>();
        box->SetCenter(position, size);
        box->SetColor(glm::vec3(color_dist(rng), color_dist(rng), color_dist(rng)));
        box->SetEdgeColor(glm::vec3(color_dist(rng) * 0.7f, color_dist(rng) * 0.7f, color_dist(rng) * 0.7f));
        
        std::string name = "grid_box_" + std::to_string(i) + "_" + std::to_string(j);
        scene_manager->AddOpenGLObject(name, std::move(box));
      }
    }

    // Nested boxes for layered selection testing
    const int nested_count = 3;
    const glm::vec3 nested_center(-10.0f, 0.0f, -6.0f);
    
    for (int i = 0; i < nested_count; ++i) {
      float scale = 2.5f - i * 0.7f;
      glm::vec3 size(scale, scale * 0.6f, scale * 0.8f);
      
      auto box = std::make_unique<BoundingBox>();
      box->SetCenter(nested_center, size);
      
      // Different rendering modes for each layer
      if (i == 0) {
        box->SetColor(glm::vec3(0.7f, 0.2f, 0.2f));  // Dark red (outer)
        box->SetRenderMode(BoundingBox::RenderMode::kSolid);
      } else if (i == 1) {
        box->SetColor(glm::vec3(0.2f, 0.7f, 0.2f));  // Dark green (middle)
        box->SetRenderMode(BoundingBox::RenderMode::kTransparent);
        box->SetOpacity(0.6f);
        box->SetShowEdges(true);
        box->SetEdgeWidth(2.0f);
      } else {
        box->SetColor(glm::vec3(0.2f, 0.2f, 0.7f));  // Dark blue (inner)
        box->SetRenderMode(BoundingBox::RenderMode::kWireframe);
        box->SetWireframeWidth(4.0f);
        box->SetShowCornerPoints(true, 6.0f);
      }
      
      std::string name = "nested_box_" + std::to_string(i);
      scene_manager->AddOpenGLObject(name, std::move(box));
    }

    // Edge case: very thin boxes
    auto thin_horizontal = std::make_unique<BoundingBox>(
        glm::vec3(-12.0f, 3.0f, -1.0f), glm::vec3(-8.0f, 3.05f, 1.0f));
    thin_horizontal->SetColor(glm::vec3(0.9f, 0.9f, 0.1f));  // Bright yellow
    thin_horizontal->SetEdgeColor(glm::vec3(0.6f, 0.6f, 0.0f));
    thin_horizontal->SetShowEdges(true);
    thin_horizontal->SetEdgeWidth(3.0f);
    scene_manager->AddOpenGLObject("thin_horizontal", std::move(thin_horizontal));

    auto thin_vertical = std::make_unique<BoundingBox>(
        glm::vec3(12.0f, -4.0f, -0.05f), glm::vec3(14.0f, 4.0f, 0.05f));
    thin_vertical->SetColor(glm::vec3(0.1f, 0.9f, 0.9f));  // Bright cyan
    thin_vertical->SetEdgeColor(glm::vec3(0.0f, 0.6f, 0.6f));
    thin_vertical->SetShowEdges(true);
    thin_vertical->SetEdgeWidth(2.0f);
    scene_manager->AddOpenGLObject("thin_vertical", std::move(thin_vertical));

    std::cout << "✓ Created complex arrangements: " << (grid_size * grid_size) 
              << " grid boxes, " << nested_count << " nested boxes, 2 thin boxes" << std::endl;
  }
};

int main() {
  BoundingBoxSelectionTest app;
  return app.Run();
}