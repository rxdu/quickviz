/**
 * @file test_point_picking_simple.cpp
 * @date 2025-08-25
 * @brief Simple test application for point cloud picking using GlView framework
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <random>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/text3d.hpp"

#include "visualization/selection/point_cloud_selector.hpp"

using namespace quickviz;
using namespace quickviz::visualization;

class PointPickingDemo {
 public:
  PointPickingDemo() = default;

  void SetupScene(GlSceneManager* scene_manager) {
    scene_manager_ = scene_manager;
    
    // Create point cloud with test data
    point_cloud_ = std::make_shared<PointCloud>();
    GenerateTestPointCloud();
    
    // Create selector
    selector_ = std::make_unique<PointCloudSelector>();
    selector_->SetPointCloud(point_cloud_);
    
    // Set selection callback
    selector_->SetSelectionCallback([this](const std::vector<size_t>& indices) {
      OnSelectionChanged(indices);
    });
    
    // Add point cloud to scene
    scene_manager_->AddOpenGLObject("point_cloud", point_cloud_);
    
    // Add pick indicator sphere (initially hidden)
    pick_indicator_ = std::make_shared<Sphere>(glm::vec3(0), 0.05f);
    pick_indicator_->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    pick_indicator_->SetVisible(false);
    scene_manager_->AddOpenGLObject("pick_indicator", pick_indicator_);
    
    // Add selection info text
    info_text_ = std::make_shared<Text3d>("No selection", 0.3f);
    info_text_->SetPosition(glm::vec3(-4.0f, 3.0f, 0.0f));
    info_text_->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
    scene_manager_->AddOpenGLObject("info_text", info_text_);
    
    std::cout << "\n=== Point Cloud Picking Demo ===" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Left Click: Pick point" << std::endl;
    std::cout << "  Shift + Left Click: Add to selection" << std::endl;
    std::cout << "  Ctrl + Left Click: Remove from selection" << std::endl;
    std::cout << "  Right Click: Clear selection" << std::endl;
    std::cout << "  S: Sphere selection (0.5 radius around last pick)" << std::endl;
    std::cout << "  B: Box selection (1x1x1 around last pick)" << std::endl;
    std::cout << "  C: Clear selection" << std::endl;
    std::cout << "  Space: Print selection statistics" << std::endl;
    std::cout << "================================\n" << std::endl;
  }

  void GenerateTestPointCloud() {
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
    
    point_cloud_->SetPoints(points, PointCloud::ColorMode::kScalarField);
    point_cloud_->SetPointSize(3.0f);
    point_cloud_->SetScalarRange(0.0f, 1.0f);
    
    std::cout << "Generated " << points.size() << " test points" << std::endl;
  }

  // Simple mouse handling - this would need to be integrated with actual input system
  void HandleClick(const glm::vec3& ray_origin, const glm::vec3& ray_direction, int mods) {
    Ray ray(ray_origin, ray_direction);
    
    // Perform point picking
    auto result = selector_->PickPoint(ray, 0.2f); // 0.2 units tolerance
    
    if (result.has_value()) {
      // Update pick indicator
      pick_indicator_->SetPosition(result->point);
      pick_indicator_->SetVisible(true);
      last_pick_position_ = result->point;
      
      // Update selection based on modifier keys
      SelectionMode mode = SelectionMode::kSingle;
      if (mods & 0x01) { // Shift
        mode = SelectionMode::kAdditive;
      } else if (mods & 0x02) { // Ctrl
        mode = SelectionMode::kSubtractive;
      } else if (mods & 0x04) { // Alt
        mode = SelectionMode::kToggle;
      }
      
      selector_->UpdateSelection({result->point_index}, mode);
      selector_->ApplySelectionVisualization("selection", 
                                            glm::vec3(1.0f, 1.0f, 0.0f), 1.5f);
      
      // Print pick info
      std::cout << "Picked point " << result->point_index 
                << " at (" << result->point.x << ", " 
                << result->point.y << ", " << result->point.z << ")"
                << " distance: " << result->distance << std::endl;
      
      UpdateInfoText();
    }
  }

  void HandleKeyPress(int key, int mods) {
    switch (key) {
      case 'S':
      case 's':
        // Sphere selection around last picked point
        if (last_pick_position_.has_value()) {
          auto indices = selector_->SelectInSphere(last_pick_position_.value(), 0.5f);
          selector_->UpdateSelection(indices, 
            (mods & 0x01) ? SelectionMode::kAdditive : SelectionMode::kSingle); // Shift
          selector_->ApplySelectionVisualization();
          UpdateInfoText();
          std::cout << "Sphere selection: " << indices.size() << " points" << std::endl;
        }
        break;
        
      case 'B':
      case 'b':
        // Box selection around last picked point
        if (last_pick_position_.has_value()) {
          glm::vec3 center = last_pick_position_.value();
          glm::vec3 half_size(0.5f);
          auto indices = selector_->SelectInBox(center - half_size, center + half_size);
          selector_->UpdateSelection(indices,
            (mods & 0x01) ? SelectionMode::kAdditive : SelectionMode::kSingle); // Shift
          selector_->ApplySelectionVisualization();
          UpdateInfoText();
          std::cout << "Box selection: " << indices.size() << " points" << std::endl;
        }
        break;
        
      case 'C':
      case 'c':
        // Clear selection
        selector_->ClearSelection();
        selector_->ApplySelectionVisualization();
        pick_indicator_->SetVisible(false);
        UpdateInfoText();
        std::cout << "Selection cleared" << std::endl;
        break;
        
      case ' ':
        // Print selection statistics
        PrintSelectionStats();
        break;
    }
  }

  void HandleRightClick() {
    // Clear selection on right click
    selector_->ClearSelection();
    selector_->ApplySelectionVisualization();
    pick_indicator_->SetVisible(false);
    UpdateInfoText();
    std::cout << "Selection cleared" << std::endl;
  }

  void OnSelectionChanged(const std::vector<size_t>& indices) {
    std::cout << "Selection changed: " << indices.size() << " points selected" << std::endl;
  }

  void UpdateInfoText() {
    size_t count = selector_->GetSelectionCount();
    if (count == 0) {
      info_text_->SetText("No selection");
    } else {
      std::string text = std::to_string(count) + " points selected";
      info_text_->SetText(text);
      
      // Show centroid
      if (count > 1) {
        glm::vec3 centroid = selector_->GetSelectionCentroid();
        std::cout << "Selection centroid: (" 
                  << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;
      }
    }
  }

  void PrintSelectionStats() {
    size_t count = selector_->GetSelectionCount();
    if (count > 0) {
      auto [min_pt, max_pt] = selector_->GetSelectionBounds();
      glm::vec3 centroid = selector_->GetSelectionCentroid();
      
      std::cout << "\n=== Selection Statistics ===" << std::endl;
      std::cout << "Count: " << count << " points" << std::endl;
      std::cout << "Centroid: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;
      std::cout << "Min bound: (" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ")" << std::endl;
      std::cout << "Max bound: (" << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << ")" << std::endl;
      std::cout << "========================\n" << std::endl;
    } else {
      std::cout << "No points selected" << std::endl;
    }
  }

 private:
  GlSceneManager* scene_manager_ = nullptr;
  std::shared_ptr<PointCloud> point_cloud_;
  std::unique_ptr<PointCloudSelector> selector_;
  std::shared_ptr<Sphere> pick_indicator_;
  std::shared_ptr<Text3d> info_text_;
  std::optional<glm::vec3> last_pick_position_;
};

int main(int argc, char* argv[]) {
  try {
    // Create the demo instance
    PointPickingDemo demo;
    
    // Configure GlView
    GlView::Config config;
    config.window_title = "Point Cloud Picking Test";
    config.show_grid = true;
    config.show_coordinate_frame = true;
    config.grid_size = 20.0f;
    config.grid_step = 1.0f;
    config.coordinate_frame_size = 2.0f;
    
    // Create GlView
    GlView view(config);
    
    // Set up scene
    view.SetSceneSetup([&demo](GlSceneManager* scene_manager) {
      demo.SetupScene(scene_manager);
    });
    
    // Add help text
    view.AddHelpSection("Point Picking", {
      "Left Click: Pick point",
      "Shift + Left Click: Add to selection", 
      "Ctrl + Left Click: Remove from selection",
      "Right Click: Clear selection"
    });
    
    view.AddHelpSection("Selection Tools", {
      "S: Sphere selection (0.5 radius)",
      "B: Box selection (1x1x1)",
      "C: Clear selection",
      "Space: Print statistics"
    });
    
    view.SetDescription(
      "Interactive point cloud picking demo.\n"
      "This demonstrates the visualization module's selection capabilities."
    );
    
    // Run the view
    view.Run();
    
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
  
  return 0;
}