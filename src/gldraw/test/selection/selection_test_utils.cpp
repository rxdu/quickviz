/**
 * @file selection_test_utils.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Implementation of shared selection testing utilities
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "selection_test_utils.hpp"

#include <cstdio>
#include <cmath>
#include <algorithm>

#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/line_strip.hpp"
#include "gldraw/renderable/cylinder.hpp"
#include "gldraw/renderable/mesh.hpp"

namespace quickviz {
namespace selection_test_utils {

// =================================================================
// SelectionInfoPanel Implementation
// =================================================================

SelectionInfoPanel::SelectionInfoPanel(const std::string& title, GlScenePanel& scene_panel)
    : Panel(title), scene_panel_(scene_panel) {}

void SelectionInfoPanel::SetLastSelection(const SelectionResult& result) {
  last_selection_ = result;
}

void SelectionInfoPanel::UpdateMultiSelection(const MultiSelection& multi) {
  multi_selection_count_ = multi.Count();
  point_count_ = multi.GetPoints().size();
  object_count_ = multi.GetObjects().size();
  if (multi_selection_count_ > 0) {
    selection_centroid_ = multi.GetCentroid();
  } else {
    selection_centroid_ = glm::vec3(0.0f);
  }
}

void SelectionInfoPanel::Draw() {
  Begin();
  DrawSelectionInfo();
  DrawMultiSelectionInfo();
  DrawSelectionControls();
  End();
}

void SelectionInfoPanel::DrawSelectionInfo() {
  ImGui::Text("SelectionManager Demo");
  ImGui::Separator();

  ImGui::Text("Current Selection:");
  if (!IsEmpty(last_selection_)) {
    if (std::holds_alternative<PointSelection>(last_selection_)) {
      auto point_selection = std::get<PointSelection>(last_selection_);
      ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "  Type: POINT");
      ImGui::Text("  Cloud: %s", point_selection.cloud_name.c_str());
      ImGui::Text("  Index: %zu", point_selection.point_index);
      ImGui::Text("  Position: (%.2f, %.2f, %.2f)", 
                  point_selection.world_position.x,
                  point_selection.world_position.y, 
                  point_selection.world_position.z);
    } else if (std::holds_alternative<ObjectSelection>(last_selection_)) {
      auto object_selection = std::get<ObjectSelection>(last_selection_);
      ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.2f, 1.0f), "  Type: OBJECT");
      ImGui::Text("  Name: %s", object_selection.object_name.c_str());
      ImGui::Text("  Position: (%.2f, %.2f, %.2f)",
                  object_selection.world_position.x,
                  object_selection.world_position.y,
                  object_selection.world_position.z);
    }
  } else {
    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "  None");
  }
}

void SelectionInfoPanel::DrawMultiSelectionInfo() {
  ImGui::Separator();
  ImGui::Text("Multi-Selection:");
  ImGui::Text("  Total Items: %zu", multi_selection_count_);
  if (multi_selection_count_ > 0) {
    ImGui::Text("  Points: %zu", point_count_);
    ImGui::Text("  Objects: %zu", object_count_);
    ImGui::Text("  Centroid: (%.2f, %.2f, %.2f)", 
                selection_centroid_.x, selection_centroid_.y, selection_centroid_.z);
  }
}

void SelectionInfoPanel::DrawSelectionControls() {
  ImGui::Separator();
  ImGui::Text("Controls:");
  ImGui::Text("  Left Click: Select");
  ImGui::Text("  Ctrl+Shift+Click: Add to selection");
  ImGui::Text("  Ctrl+Alt+Click: Toggle selection");
  ImGui::Text("  Ctrl+Right Click: Clear selection");
  
  ImGui::Separator();
  ImGui::Text("Keyboard Shortcuts:");
  ImGui::Text("  P: Points only");
  ImGui::Text("  O: Objects only");
  ImGui::Text("  H: Hybrid mode");
  ImGui::Text("  C: Clear selection");
}

// =================================================================
// SelectionDemoPanel Implementation
// =================================================================

SelectionDemoPanel::SelectionDemoPanel(const std::string& title)
    : GlScenePanel(title) {}

void SelectionDemoPanel::SetSelectionCallback(std::function<void(const SelectionResult&, const MultiSelection&)> callback) {
  selection_callback_ = callback;
  
  // Set up the internal SelectionManager callback
  GetSelection().SetSelectionCallback([this](const SelectionResult& result, const MultiSelection& multi) {
    // Call external callback first
    if (selection_callback_) {
      selection_callback_(result, multi);
    }
    
    // Print selection to console for debugging
    if (!IsEmpty(result)) {
      if (std::holds_alternative<PointSelection>(result)) {
        auto point_sel = std::get<PointSelection>(result);
        std::cout << "[POINT] Cloud: " << point_sel.cloud_name 
                  << ", Index: " << point_sel.point_index << std::endl;
      } else if (std::holds_alternative<ObjectSelection>(result)) {
        auto obj_sel = std::get<ObjectSelection>(result);
        std::cout << "[OBJECT] Name: " << obj_sel.object_name << std::endl;
      }
    }
  });
}

void SelectionDemoPanel::Draw() {
  // Handle custom mouse input first
  HandleMouseSelection();
  
  // Then draw normally
  GlScenePanel::Draw();
  HandleKeyboardShortcuts();
}

void SelectionDemoPanel::HandleMouseSelection() {
  if (!ImGui::IsWindowHovered()) return;

  ImGuiIO& io = ImGui::GetIO();

  // Handle mouse clicks with modifiers for selection
  if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    ImVec2 mouse_pos = ImGui::GetMousePos();
    ImVec2 window_pos = ImGui::GetWindowPos();
    ImVec2 window_content_min = ImGui::GetWindowContentRegionMin();

    // Convert to relative coordinates
    float relative_x = mouse_pos.x - window_pos.x - window_content_min.x;
    float relative_y = mouse_pos.y - window_pos.y - window_content_min.y;

    SelectionOptions options;
    options.radius = 5;              // 5-pixel tolerance for easier selection
    options.mode = selection_mode_;  // Use current selection mode

    if (io.KeyCtrl) {
      if (io.KeyShift) {
        // Ctrl+Shift+Click: Add to selection
        AddToSelection(relative_x, relative_y, options);
      } else if (io.KeyAlt) {
        // Ctrl+Alt+Click: Toggle selection
        GetSelection().ToggleSelection(relative_x, relative_y, options);
      } else {
        // Ctrl+Click: Single selection (replace)
        Select(relative_x, relative_y, options);
      }
    }
  }

  // Clear selection with Ctrl+Right Click
  if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) && io.KeyCtrl) {
    ClearSelection();
    std::cout << "Selection cleared (Ctrl+Right Click)" << std::endl;
  }
}

void SelectionDemoPanel::HandleKeyboardShortcuts() {
  // Selection mode shortcuts
  if (ImGui::IsKeyPressed(ImGuiKey_P)) {
    selection_mode_ = SelectionMode::kPoints;
    std::cout << "Selection mode: POINTS only" << std::endl;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_O)) {
    selection_mode_ = SelectionMode::kObjects;
    std::cout << "Selection mode: OBJECTS only" << std::endl;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_H)) {
    selection_mode_ = SelectionMode::kHybrid;
    std::cout << "Selection mode: HYBRID (both)" << std::endl;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_C)) {
    ClearSelection();
    std::cout << "Selection cleared" << std::endl;
  }
}

// =================================================================
// SelectionTestApp Implementation
// =================================================================

SelectionTestApp::SelectionTestApp(const std::string& title) : title_(title) {
  viewer_ = std::make_unique<Viewer>(title);
  scene_panel_ = std::make_shared<SelectionDemoPanel>("3D Scene");
  info_panel_ = std::make_shared<SelectionInfoPanel>("Selection Info", *scene_panel_);
  main_container_ = std::make_shared<Box>("main_container");
  
  // Setup panel properties
  scene_panel_->SetAutoLayout(true);
  scene_panel_->SetFlexGrow(1.0f);
  scene_panel_->SetBackgroundColor(0.1f, 0.1f, 0.2f, 1.0f);
  scene_panel_->SetShowRenderingInfo(true);  // Show FPS overlay

  info_panel_->SetAutoLayout(true);
  info_panel_->SetFlexBasis(250.0f);
  info_panel_->SetFlexGrow(0.0f);
  info_panel_->SetFlexShrink(0.0f);
  
  // Connect scene panel to info panel via callback (like original design)
  scene_panel_->SetSelectionCallback(
      [info_panel_ptr = info_panel_.get()](const SelectionResult& result, const MultiSelection& multi) {
        info_panel_ptr->SetLastSelection(result);
        info_panel_ptr->UpdateMultiSelection(multi);
      });
  
  // Setup container layout
  main_container_->SetFlexDirection(Styling::FlexDirection::kRow);
  main_container_->AddChild(info_panel_);
  main_container_->AddChild(scene_panel_);
  
  viewer_->AddSceneObject(main_container_);
}

void SelectionTestApp::AddReferenceGrid(GlSceneManager* scene_manager, float size, float spacing) {
  auto grid = std::make_unique<Grid>(size, spacing, glm::vec3(0.3f, 0.3f, 0.3f));
  scene_manager->AddOpenGLObject("reference_grid", std::move(grid));
}

void SelectionTestApp::PrintTestHeader() {
  std::cout << "=== " << title_ << " ===" << std::endl;
  std::cout << GetTestDescription() << std::endl;
  std::cout << std::endl;
}

void SelectionTestApp::PrintInstructions() {
  std::cout << "=== Instructions ===" << std::endl;
  std::cout << GetInstructions() << std::endl;
  std::cout << std::endl;
}

int SelectionTestApp::Run() {
  try {
    PrintTestHeader();
    
    // Setup test objects
    SetupTestObjects(scene_panel_->GetSceneManager());
    
    // Add reference grid
    AddReferenceGrid(scene_panel_->GetSceneManager());
    
    PrintInstructions();
    
    std::cout << "✓ Test setup complete! Starting interactive session..." << std::endl;
    
    // Run the application
    viewer_->Show();
    
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
}

// =================================================================
// TestObjectFactory Implementation
// =================================================================

namespace TestObjectFactory {

void CreateSphereGrid(GlSceneManager* scene_manager,
                     const std::vector<glm::vec3>& positions,
                     const std::string& prefix,
                     float radius) {
  std::cout << "Creating sphere grid with " << positions.size() << " spheres:" << std::endl;
  
  for (size_t i = 0; i < positions.size(); ++i) {
    auto sphere = std::make_unique<Sphere>(positions[i], radius);
    
    // Generate systematic colors for visual identification
    glm::vec3 color;
    float hue = (i * 137.5f) / 360.0f; // Golden angle for good distribution
    hue = hue - floor(hue); // Keep in [0,1] range
    
    if (hue < 0.33f) {
      color = glm::vec3(1.0f - 3.0f * hue, 3.0f * hue, 0.2f);
    } else if (hue < 0.67f) {
      float h = hue - 0.33f;
      color = glm::vec3(0.2f, 1.0f - 3.0f * h, 3.0f * h);
    } else {
      float h = hue - 0.67f;
      color = glm::vec3(3.0f * h, 0.2f, 1.0f - 3.0f * h);
    }
    
    sphere->SetColor(color);
    sphere->SetRenderMode(Sphere::RenderMode::kSolid);

    // Create unique name with zero-padding
    char name_buffer[64];
    snprintf(name_buffer, sizeof(name_buffer), "%s_%02zu", prefix.c_str(), i);
    std::string name(name_buffer);

    std::cout << "  " << name << " at (" << positions[i].x << ", " 
              << positions[i].y << ", " << positions[i].z << ")" << std::endl;

    scene_manager->AddOpenGLObject(name, std::move(sphere));
  }
}

void CreateTestPointClouds(GlSceneManager* scene_manager) {
  // Create grid pattern point cloud
  std::vector<glm::vec3> grid_points;
  std::vector<glm::vec3> grid_colors;
  
  const int grid_size = 10;
  const float spacing = 0.5f;
  for (int y = 0; y < grid_size; ++y) {
    for (int x = 0; x < grid_size; ++x) {
      float px = (x - grid_size / 2) * spacing;
      float py = (y - grid_size / 2) * spacing;
      grid_points.emplace_back(px, py, 2.0f);

      // Checkerboard pattern colors
      if ((x + y) % 2 == 0) {
        grid_colors.emplace_back(0.2f, 1.0f, 0.2f);  // Bright green
      } else {
        grid_colors.emplace_back(0.2f, 0.8f, 0.8f);  // Cyan
      }
    }
  }

  auto grid_cloud = std::make_unique<PointCloud>();
  grid_cloud->SetPoints(grid_points, grid_colors);
  grid_cloud->SetPointSize(8.0f);
  scene_manager->AddOpenGLObject("grid_points", std::move(grid_cloud));

  // Create spiral pattern point cloud
  std::vector<glm::vec3> spiral_points;
  std::vector<glm::vec3> spiral_colors;

  const int spiral_count = 50;
  const float spiral_radius = 3.0f;
  const float spiral_height_start = 3.0f;
  const float spiral_height_range = 2.0f;

  for (int i = 0; i < spiral_count; ++i) {
    float t = static_cast<float>(i) / spiral_count;
    float angle = t * 6.28318f * 3.0f;                 // 3 full rotations
    float radius = spiral_radius * (0.3f + 0.7f * t);  // Expanding spiral
    float height = spiral_height_start + spiral_height_range * t;

    spiral_points.emplace_back(radius * cos(angle), radius * sin(angle), height);

    // Rainbow colors
    float hue = t * 360.0f;
    if (hue < 120.0f) {
      float f = hue / 120.0f;
      spiral_colors.emplace_back(1.0f - f, f, 0.2f);
    } else if (hue < 240.0f) {
      float f = (hue - 120.0f) / 120.0f;
      spiral_colors.emplace_back(0.2f, 1.0f - f, f);
    } else {
      float f = (hue - 240.0f) / 120.0f;
      spiral_colors.emplace_back(f, 0.2f, 1.0f - f);
    }
  }

  auto spiral_cloud = std::make_unique<PointCloud>();
  spiral_cloud->SetPoints(spiral_points, spiral_colors);
  spiral_cloud->SetPointSize(6.0f);
  scene_manager->AddOpenGLObject("spiral_points", std::move(spiral_cloud));

  std::cout << "✓ Created test point clouds:" << std::endl;
  std::cout << "  - Grid pattern: " << grid_points.size() << " points" << std::endl;
  std::cout << "  - Spiral pattern: " << spiral_points.size() << " points" << std::endl;
}

} // namespace TestObjectFactory

// =================================================================
// TestHelpers Implementation
// =================================================================

namespace TestHelpers {

std::vector<glm::vec3> GenerateTestColors(size_t count, float base_hue) {
  std::vector<glm::vec3> colors;
  colors.reserve(count);
  
  for (size_t i = 0; i < count; ++i) {
    float hue = base_hue + (i * 360.0f / count);
    hue = fmod(hue, 360.0f) / 360.0f; // Normalize to [0,1]
    
    // Convert HSV to RGB (simplified)
    glm::vec3 color;
    if (hue < 0.33f) {
      color = glm::vec3(1.0f - 3.0f * hue, 3.0f * hue, 0.2f);
    } else if (hue < 0.67f) {
      float h = hue - 0.33f;
      color = glm::vec3(0.2f, 1.0f - 3.0f * h, 3.0f * h);
    } else {
      float h = hue - 0.67f;
      color = glm::vec3(3.0f * h, 0.2f, 1.0f - 3.0f * h);
    }
    
    colors.push_back(color);
  }
  
  return colors;
}

std::vector<glm::vec3> GenerateGridPositions(const glm::ivec3& dimensions,
                                            float spacing,
                                            const glm::vec3& center) {
  std::vector<glm::vec3> positions;
  positions.reserve(dimensions.x * dimensions.y * dimensions.z);
  
  glm::vec3 start_offset = center - glm::vec3(dimensions - 1) * spacing * 0.5f;
  
  for (int z = 0; z < dimensions.z; ++z) {
    for (int y = 0; y < dimensions.y; ++y) {
      for (int x = 0; x < dimensions.x; ++x) {
        glm::vec3 pos = start_offset + glm::vec3(x, y, z) * spacing;
        positions.push_back(pos);
      }
    }
  }
  
  return positions;
}

void PrintObjectDetails(const std::vector<std::pair<std::string, glm::vec3>>& objects) {
  std::cout << "Object Details:" << std::endl;
  for (const auto& obj : objects) {
    std::cout << "  " << obj.first << " at (" << obj.second.x 
              << ", " << obj.second.y << ", " << obj.second.z << ")" << std::endl;
  }
}

} // namespace TestHelpers

} // namespace selection_test_utils
} // namespace quickviz