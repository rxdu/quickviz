/*
 * test_layer_system_demo.cpp
 *
 * Created on: Jan 2025
 * Description: Clear demonstration of the multi-layer rendering system
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <glm/glm.hpp>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/layer_manager.hpp"

using namespace quickviz;

class LayerSystemDemo {
 public:
  LayerSystemDemo() {
    SetupViewer();
    CreateStructuredPointCloud();
    SetupLayers();
  }

  void Run() { viewer_.Show(); }

 private:
  void SetupViewer() {
    // Create main container
    auto main_box = std::make_shared<Box>("main_box");
    main_box->SetFlexDirection(Styling::FlexDirection::kRow);

    // Create 3D scene
    scene_manager_ = std::make_shared<GlSceneManager>("Layer System Demo");
    scene_manager_->SetAutoLayout(true);
    scene_manager_->SetNoTitleBar(true);
    scene_manager_->SetFlexGrow(1.0f);

    main_box->AddChild(scene_manager_);
    viewer_.AddSceneObject(main_box);
  }

  void CreateStructuredPointCloud() {
    // Create a structured point cloud that forms 3 overlapping circles
    // This makes it easy to see which points belong to which layer

    const float radius = 5.0f;
    const int points_per_circle = 100;

    // Circle 1: Center at (-3, 0, 0)
    for (int i = 0; i < points_per_circle; ++i) {
      float angle = 2.0f * M_PI * i / points_per_circle;
      float x = -3.0f + radius * std::cos(angle);
      float y = 0.0f;
      float z = radius * std::sin(angle);
      float intensity = 0.3f;  // Low intensity = blue
      base_points_.push_back(glm::vec4(x, y, z, intensity));
      circle1_indices_.push_back(base_points_.size() - 1);
    }

    // Circle 2: Center at (3, 0, 0)
    for (int i = 0; i < points_per_circle; ++i) {
      float angle = 2.0f * M_PI * i / points_per_circle;
      float x = 3.0f + radius * std::cos(angle);
      float y = 0.0f;
      float z = radius * std::sin(angle);
      float intensity = 0.5f;  // Medium intensity = green/yellow
      base_points_.push_back(glm::vec4(x, y, z, intensity));
      circle2_indices_.push_back(base_points_.size() - 1);
    }

    // Circle 3: Center at (0, 0, 3) - overlaps with both circles
    for (int i = 0; i < points_per_circle; ++i) {
      float angle = 2.0f * M_PI * i / points_per_circle;
      float x = radius * std::cos(angle);
      float y = 0.0f;
      float z = 3.0f + radius * std::sin(angle);
      float intensity = 0.7f;  // High intensity = orange/red
      base_points_.push_back(glm::vec4(x, y, z, intensity));
      circle3_indices_.push_back(base_points_.size() - 1);
    }

    // Add some scattered points in the middle for additional visual interest
    for (int i = 0; i < 50; ++i) {
      float angle = 2.0f * M_PI * i / 50;
      float r = 2.0f;
      float x = r * std::cos(angle);
      float y = 0.0f;
      float z = r * std::sin(angle);
      float intensity = 0.1f;
      base_points_.push_back(glm::vec4(x, y, z, intensity));
      center_indices_.push_back(base_points_.size() - 1);
    }

    std::cout << "\n=== Created Structured Point Cloud ===" << std::endl;
    std::cout << "Total points: " << base_points_.size() << std::endl;
    std::cout << "Circle 1 (left): " << circle1_indices_.size() << " points"
              << std::endl;
    std::cout << "Circle 2 (right): " << circle2_indices_.size() << " points"
              << std::endl;
    std::cout << "Circle 3 (front): " << circle3_indices_.size() << " points"
              << std::endl;
    std::cout << "Center points: " << center_indices_.size() << " points"
              << std::endl;
  }

  void SetupLayers() {
    // Create point cloud with explicit colors
    // Convert vec4 points to vec3 and create base colors
    std::vector<glm::vec3> points_3d;
    std::vector<glm::vec3> base_colors;

    for (const auto& point : base_points_) {
      points_3d.push_back(glm::vec3(point.x, point.y, point.z));
      base_colors.push_back(
          glm::vec3(0.7f, 0.7f, 0.7f));  // Light gray base color
    }

    point_cloud_ = std::make_unique<PointCloud>();
    point_cloud_->SetPoints(points_3d, base_colors);
    point_cloud_->SetPointSize(8.0f);
    point_cloud_->SetRenderMode(PointMode::kSphere);

    // Create layers with different priorities
    // Higher priority layers render on top

    // Layer 1: Highlight Circle 1 in RED (Priority 50)
    auto layer1 = point_cloud_->CreateLayer("circle1_highlight", 50);
    if (layer1) {
      layer1->SetPoints(circle1_indices_);
      layer1->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));  // RED
      layer1->SetPointSizeMultiplier(1.2f);  // Double size to test if size change works
      layer1->SetHighlightMode(PointLayer::HighlightMode::kColorAndSize);
      layer1->SetVisible(false);  // Start disabled
    }

    // Layer 2: Highlight Circle 2 in GREEN (Priority 60)
    auto layer2 = point_cloud_->CreateLayer("circle2_highlight", 60);
    if (layer2) {
      layer2->SetPoints(circle2_indices_);
      layer2->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));  // GREEN
      layer2->SetPointSizeMultiplier(1.2f);  // Slightly larger to cover base points
      layer2->SetHighlightMode(PointLayer::HighlightMode::kSphereSurface);
      layer2->SetVisible(false);  // Start disabled
    }

    // Layer 3: Highlight Circle 3 in BLUE (Priority 70)
    auto layer3 = point_cloud_->CreateLayer("circle3_highlight", 70);
    if (layer3) {
      layer3->SetPoints(circle3_indices_);
      layer3->SetColor(glm::vec3(0.0f, 0.5f, 1.0f));  // BLUE
      layer3->SetPointSizeMultiplier(1.2f);  // Slightly larger to cover base points
      layer3->SetHighlightMode(PointLayer::HighlightMode::kSphereSurface);
      layer3->SetVisible(false);  // Start disabled
    }

    // Layer 4: Selection layer - overlapping region (Priority 100 - highest)
    std::vector<size_t> overlap_indices;
    // Find points in overlapping regions (simplified - just take some from each
    // circle)
    for (int i = 40; i < 60; ++i) {
      if (i < circle1_indices_.size())
        overlap_indices.push_back(circle1_indices_[i]);
      if (i < circle2_indices_.size())
        overlap_indices.push_back(circle2_indices_[i]);
      if (i < circle3_indices_.size())
        overlap_indices.push_back(circle3_indices_[i]);
    }

    auto selection_layer = point_cloud_->CreateLayer("selection", 100);
    if (selection_layer) {
      selection_layer->SetPoints(overlap_indices);
      selection_layer->SetColor(glm::vec3(1.2f, 1.0f, 0.0f));  // YELLOW
      selection_layer->SetPointSizeMultiplier(2.0f);
      selection_layer->SetHighlightMode(
          PointLayer::HighlightMode::kSphereSurface);
      selection_layer->SetVisible(false);  // Start disabled
    }

    // Add grid for reference
    auto grid =
        std::make_unique<Grid>(20.0f, 2.0f, glm::vec3(0.5f, 0.5f, 0.5f));

    // Add to scene
    scene_manager_->AddOpenGLObject("point_cloud", std::move(point_cloud_));
    scene_manager_->AddOpenGLObject("grid", std::move(grid));

    point_cloud_ptr_ = static_cast<PointCloud*>(
        scene_manager_->GetOpenGLObject("point_cloud"));

    std::cout << "\n=== Layer Configuration ===" << std::endl;
    std::cout << "Created 4 layers (all initially disabled):" << std::endl;
    std::cout << "1. Circle 1 (RED) - Priority 50" << std::endl;
    std::cout << "2. Circle 2 (GREEN) - Priority 60" << std::endl;
    std::cout << "3. Circle 3 (BLUE) - Priority 70" << std::endl;
    std::cout << "4. Selection (YELLOW) - Priority 100 (highest) - Point "
                 "indices 40-60 from each circle"
              << std::endl;

    layer1_enabled_ = false;
    layer2_enabled_ = false;
    layer3_enabled_ = false;
    selection_enabled_ = false;
  }

  void EnableDemoLayers() {
    // Enable layers to show the effect
    std::cout << "\n=== Enabling Demo Layers ===" << std::endl;

    // Enable circle 1 (red)
    auto layer1 = point_cloud_ptr_->GetLayer("circle1_highlight");
    if (layer1) {
      layer1->SetVisible(true);
      std::cout << "✓ Enabled Circle 1 (Red) layer" << std::endl;
    }

    // Enable circle 2 (green)
    auto layer2 = point_cloud_ptr_->GetLayer("circle2_highlight");
    if (layer2) {
      layer2->SetVisible(true);
      std::cout << "✓ Enabled Circle 2 (Green) layer" << std::endl;
    }

    // Enable circle 3 (blue)
    auto layer3 = point_cloud_ptr_->GetLayer("circle3_highlight");
    if (layer3) {
      layer3->SetVisible(true);
      std::cout << "✓ Enabled Circle 3 (Blue) layer" << std::endl;
    }

    // Enable selection (yellow)
    auto selection = point_cloud_ptr_->GetLayer("selection");
    if (selection) {
      selection->SetVisible(true);
      std::cout << "✓ Enabled Selection (Yellow) layer" << std::endl;
    }

    std::cout << "\nNote: Yellow has highest priority and highlights specific "
                 "point ranges from each circle"
              << std::endl;
  }

 public:
  void Show() {
    EnableDemoLayers();

    std::cout << "\n=== Layer System Demonstration ===" << std::endl;
    std::cout << "You should see 3 colored circles of points:" << std::endl;
    std::cout << "- LEFT circle: RED (Priority 50)" << std::endl;
    std::cout << "- RIGHT circle: GREEN (Priority 60)" << std::endl;
    std::cout << "- FRONT circle: BLUE (Priority 70)" << std::endl;
    std::cout << "- Selected points: YELLOW (Priority 100 - highest)"
              << std::endl;
    std::cout
        << "\nThe yellow points demonstrate the priority system by highlighting"
        << std::endl;
    std::cout
        << "specific point ranges (indices 40-60) from each circle with the"
        << std::endl;
    std::cout << "highest priority color that overrides lower priority layers."
              << std::endl;
    std::cout << "\nCamera controls:" << std::endl;
    std::cout << "- Left mouse: Rotate" << std::endl;
    std::cout << "- Middle mouse: Pan" << std::endl;
    std::cout << "- Scroll: Zoom" << std::endl;

    viewer_.Show();
  }

 private:
  Viewer viewer_;
  std::shared_ptr<GlSceneManager> scene_manager_;
  std::unique_ptr<PointCloud> point_cloud_;
  PointCloud* point_cloud_ptr_ = nullptr;  // Raw pointer for access after move

  std::vector<glm::vec4> base_points_;
  std::vector<size_t> circle1_indices_;
  std::vector<size_t> circle2_indices_;
  std::vector<size_t> circle3_indices_;
  std::vector<size_t> center_indices_;

  bool layer1_enabled_ = false;
  bool layer2_enabled_ = false;
  bool layer3_enabled_ = false;
  bool selection_enabled_ = false;
};

int main() {
  try {
    std::cout << "=== QuickViz Layer System Demo ===" << std::endl;
    std::cout << "This demo shows how the multi-layer rendering system works"
              << std::endl;
    std::cout << "with clear, structured point arrangements.\n" << std::endl;

    LayerSystemDemo demo;
    demo.Show();

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}