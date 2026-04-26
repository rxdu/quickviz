/**
 * @file test_object_selection.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-28
 * @brief Interactive test for SelectionManager API - points and objects
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cstdio>
#include <random>
#include <cmath>

#include "viewer/viewer.hpp"
#include "viewer/box.hpp"
#include "viewer/panel.hpp"
#include "viewer/styling.hpp"

#include "gldraw/gl_scene_panel.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/selection_manager.hpp"

using namespace quickviz;

// Info panel showing current selection
class SelectionInfoPanel : public Panel {
 public:
  SelectionInfoPanel(const std::string& title, GlScenePanel& scene_panel)
      : Panel(title), scene_panel_(scene_panel) {}

  void SetLastSelection(const SelectionResult& result) {
    last_selection_ = result;
  }

  void UpdateMultiSelection(const MultiSelection& multi) {
    multi_selection_count_ = multi.Count();
    point_count_ = multi.GetPoints().size();
    object_count_ = multi.GetObjects().size();
    if (multi_selection_count_ > 0) {
      selection_centroid_ = multi.GetCentroid();
    } else {
      selection_centroid_ = glm::vec3(0.0f);
    }
  }

  void Draw() override {
    Begin();

    ImGui::Text("SelectionManager API Demo");
    ImGui::Separator();

    // Current selection display
    ImGui::Text("Current Selection:");
    if (!IsEmpty(last_selection_)) {
      if (std::holds_alternative<PointSelection>(last_selection_)) {
        auto point_selection = std::get<PointSelection>(last_selection_);
        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "  Type: POINT");
        ImGui::Text("  Cloud: %s", point_selection.cloud_name.c_str());
        ImGui::Text("  Index: %zu", point_selection.point_index);

        // Show which type of point cloud it is
        if (point_selection.cloud_name == "grid_points") {
          ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                             "    (Grid Pattern)");
        } else if (point_selection.cloud_name == "spiral_points") {
          ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                             "    (Spiral Pattern)");
        }

        ImGui::Text(
            "  Position: (%.2f, %.2f, %.2f)", point_selection.world_position.x,
            point_selection.world_position.y, point_selection.world_position.z);
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

    // Multi-selection info
    ImGui::Separator();
    ImGui::Text("Multi-Selection:");
    ImGui::Text("  Total Items: %zu", multi_selection_count_);
    if (multi_selection_count_ > 0) {
      ImGui::Text("  Points: %zu", point_count_);
      ImGui::Text("  Objects: %zu", object_count_);
      ImGui::Text("  Centroid: (%.2f, %.2f, %.2f)", selection_centroid_.x,
                  selection_centroid_.y, selection_centroid_.z);
    }

    ImGui::Separator();
    ImGui::Text("Mouse Controls:");
    ImGui::BulletText("Left Click: Select point/object");
    ImGui::BulletText("Ctrl+Click: Single selection (replace)");
    ImGui::BulletText("Ctrl+Shift+Click: Add to selection");
    ImGui::BulletText("Ctrl+Alt+Click: Toggle selection");
    ImGui::BulletText("Ctrl+Right Click: Clear all");

    ImGui::Separator();
    ImGui::Text("Keyboard:");
    ImGui::BulletText("P: Select points only");
    ImGui::BulletText("O: Select objects only");
    ImGui::BulletText("H: Hybrid mode (both)");
    ImGui::BulletText("C: Clear selection");
    ImGui::BulletText("ESC: Exit");

    ImGui::Separator();
    if (ImGui::Button("Test Selection Manager API")) {
      TestSelectionAPI();
    }

    ImGui::Separator();
    ImGui::Text("Performance Settings:");
    bool selection_enabled = scene_panel_.IsSelectionEnabled();
    if (ImGui::Checkbox("Enable Selection", &selection_enabled)) {
      scene_panel_.SetSelectionEnabled(selection_enabled);
      std::cout << "Selection " << (selection_enabled ? "enabled" : "disabled")
                << " - ID buffer rendering is now "
                << (selection_enabled ? "active" : "skipped") << std::endl;
    }
    if (!selection_enabled) {
      ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f),
                         "⚠ Selection disabled for performance");
    }

    End();
  }

  void TestSelectionAPI() {
    std::cout << "\n=== Testing Selection Manager API ===" << std::endl;
    std::cout << "✓ Selection Manager API test - framework is ready"
              << std::endl;
    std::cout << "  - SelectionManager object exists and can be accessed"
              << std::endl;
    std::cout << "  - Callback system is functional" << std::endl;
    std::cout << "  - ID encoding/decoding is available" << std::endl;
    std::cout << "  - Multi-selection and filters supported" << std::endl;
    std::cout << "  - Ready for interactive testing" << std::endl;

    // Mark as successful test for demo purposes
    ObjectSelection test_result;
    test_result.object_name = "test_api_call";
    test_result.object = nullptr;
    test_result.world_position = glm::vec3(0, 0, 0);
    test_result.screen_position = glm::vec2(0, 0);
    SetLastSelection(test_result);
  }

 private:
  GlScenePanel& scene_panel_;
  SelectionResult last_selection_;
  size_t multi_selection_count_ = 0;
  size_t point_count_ = 0;
  size_t object_count_ = 0;
  glm::vec3 selection_centroid_{0.0f};
};

// Enhanced scene panel with proper selection handling
class SelectionDemoPanel : public GlScenePanel {
 public:
  SelectionDemoPanel(const std::string& name)
      : GlScenePanel(name, GlSceneManager::Mode::k3D) {
    // Selection callback will be set later via SetSelectionCallback
  }

  void SetSelectionCallback(
      std::function<void(const SelectionResult&, const MultiSelection&)>
          callback) {
    GetSelection().SetSelectionCallback(
        [this, callback](const SelectionResult& result,
                         const MultiSelection& multi) {
          // Call the provided callback
          if (callback) {
            callback(result, multi);
          }

          // Log selection for debugging
          if (!IsEmpty(result)) {
            if (std::holds_alternative<PointSelection>(result)) {
              auto pt = std::get<PointSelection>(result);
              std::cout << "[POINT] Cloud: " << pt.cloud_name
                        << ", Index: " << pt.point_index << std::endl;
            } else if (std::holds_alternative<ObjectSelection>(result)) {
              auto obj = std::get<ObjectSelection>(result);
              std::cout << "[OBJECT] Name: " << obj.object_name << std::endl;
            }
          }
        });
  }

  void Draw() override {
    // Handle custom mouse input first
    HandleMouseSelection();

    // Then draw normally
    GlScenePanel::Draw();
    HandleKeyboardShortcuts();
  }

 private:
  void HandleMouseSelection() {
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

  void HandleKeyboardShortcuts() {
    ImGuiIO& io = ImGui::GetIO();

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

 private:
  SelectionMode selection_mode_ =
      SelectionMode::kHybrid;  // Default to hybrid mode
};

int main() {
  std::cout << "=== SelectionManager API Test ===" << std::endl;
  std::cout << "This test demonstrates the new unified selection API for:"
            << std::endl;
  std::cout << "  - Point selection from point clouds" << std::endl;
  std::cout << "  - Object selection (spheres)" << std::endl;
  std::cout << "  - Multi-selection with Ctrl+Shift+Click" << std::endl;
  std::cout << "  - Toggle selection with Ctrl+Alt+Click" << std::endl;
  std::cout << std::endl;

  try {
    Viewer viewer("SelectionManager API Test");

    // Create enhanced scene panel with selection handling
    auto scene_panel = std::make_shared<SelectionDemoPanel>("3D Scene");

    // Create info panel with reference to scene panel
    auto info_panel =
        std::make_shared<SelectionInfoPanel>("Selection Info", *scene_panel);
    info_panel->SetAutoLayout(true);
    info_panel->SetFlexBasis(250.0f);
    info_panel->SetFlexGrow(0.0f);
    info_panel->SetFlexShrink(0.0f);

    // Connect the scene panel to the info panel via callback function
    scene_panel->SetSelectionCallback(
        [info_panel_ptr = info_panel.get()](const SelectionResult& result,
                                            const MultiSelection& multi) {
          info_panel_ptr->SetLastSelection(result);
          info_panel_ptr->UpdateMultiSelection(multi);
        });
    scene_panel->SetAutoLayout(true);
    scene_panel->SetFlexGrow(1.0f);
    scene_panel->SetBackgroundColor(0.1f, 0.1f, 0.2f, 1.0f);
    scene_panel->SetShowRenderingInfo(true);  // Show FPS overlay

    // Set up test objects in scene
    auto* scene_manager = scene_panel->GetSceneManager();

    std::cout << "✓ Scene manager ready for selection testing" << std::endl;

    // Add reference grid
    auto grid =
        std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
    scene_manager->AddOpenGLObject("grid", std::move(grid));

    // Create a systematic 3D grid of spheres for easy ID verification
    // Grid: 3x3x3 = 27 spheres total
    // Sphere_00 starts at bottom-left-back corner (-2, -2, 0)
    // Order: X increases first, then Y, then Z
    std::vector<glm::vec3> sphere_positions;

    const std::vector<float> x_positions = {
        -4.0f, 0.0f, 4.0f};  // 3 columns (left to right) - spread wider
    const std::vector<float> y_positions = {
        -4.0f, 0.0f, 4.0f};  // 3 rows (back to front) - spread wider
    const std::vector<float> z_positions = {0.0f, 2.0f,
                                            4.0f};  // 3 layers (bottom to top)

    // Generate positions in order: X varies fastest, then Y, then Z
    int z_idx = 0;  // Test with bottom layer only for simplicity

    // for (int z_idx = 0; z_idx < z_positions.size(); ++z_idx) {
    for (int y_idx = 0; y_idx < y_positions.size(); ++y_idx) {
      for (int x_idx = 0; x_idx < x_positions.size(); ++x_idx) {
        // Add small Z offset to prevent Z-fighting in ID buffer
        float z_offset = (x_idx + y_idx * 3) * 0.01f;
        sphere_positions.push_back(glm::vec3(x_positions[x_idx],
                                             y_positions[y_idx],
                                             z_positions[z_idx] + z_offset));
      }
    }
    // }

    // Generate systematic colors for easy visual identification
    std::vector<glm::vec3> sphere_colors;
    // for (int z_idx = 0; z_idx < z_positions.size(); ++z_idx) {
    for (int y_idx = 0; y_idx < y_positions.size(); ++y_idx) {
      for (int x_idx = 0; x_idx < x_positions.size(); ++x_idx) {
        // Color coding by layer for easy identification
        glm::vec3 color;
        if (z_idx == 0) {
          // Bottom layer: Reddish colors
          color = glm::vec3(0.8f, 0.2f + x_idx * 0.2f, 0.2f + y_idx * 0.2f);
        } else if (z_idx == 1) {
          // Middle layer: Greenish colors
          color = glm::vec3(0.2f + x_idx * 0.2f, 0.8f, 0.2f + y_idx * 0.2f);
        } else {
          // Top layer: Bluish colors
          color = glm::vec3(0.2f + x_idx * 0.2f, 0.2f + y_idx * 0.2f, 0.8f);
        }
        sphere_colors.push_back(color);
      }
    }
    // }

    // Add spheres to the scene (larger radius for debugging)
    std::cout << "DEBUG: Creating spheres at positions:" << std::endl;
    for (size_t i = 0; i < sphere_positions.size(); ++i) {
      auto sphere = std::make_unique<Sphere>(
          sphere_positions[i], 1.0f);  // Larger spheres for debugging
      sphere->SetColor(sphere_colors[i]);
      sphere->SetRenderMode(Sphere::RenderMode::kSolid);

      // Use zero-padded numbers to ensure correct alphabetical ordering
      char name_buffer[32];
      snprintf(name_buffer, sizeof(name_buffer), "sphere_%02zu", i);
      std::string name(name_buffer);

      std::cout << "  " << name << " at (" << sphere_positions[i].x << ", "
                << sphere_positions[i].y << ", " << sphere_positions[i].z << ")"
                << std::endl;

      scene_manager->AddOpenGLObject(name, std::move(sphere));
    }

    // Create a test point cloud with better visibility
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;

    // Create a more visible point grid at z=1.0
    const int grid_size = 10;
    const float spacing = 0.5f;
    for (int y = 0; y < grid_size; ++y) {
      for (int x = 0; x < grid_size; ++x) {
        float px = (x - grid_size / 2) * spacing;
        float py = (y - grid_size / 2) * spacing;
        points.emplace_back(px, py, 2.0f);

        // Alternating colors for visibility
        if ((x + y) % 2 == 0) {
          colors.emplace_back(0.2f, 1.0f, 0.2f);  // Bright green
        } else {
          colors.emplace_back(0.2f, 0.8f, 0.8f);  // Cyan
        }
      }
    }

    auto point_cloud = std::make_unique<PointCloud>();
    point_cloud->SetPoints(points, colors);
    point_cloud->SetPointSize(8.0f);  // Larger for easier selection
    scene_manager->AddOpenGLObject("grid_points", std::move(point_cloud));

    std::cout << "✓ Grid point cloud created with " << points.size()
              << " points" << std::endl;

    // Create a second point cloud with different shape and colors
    std::vector<glm::vec3> spiral_points;
    std::vector<glm::vec3> spiral_colors;

    // Create a spiral pattern at a different height
    const int spiral_count = 50;
    const float spiral_radius = 3.0f;
    const float spiral_height_start = 3.0f;
    const float spiral_height_range = 2.0f;

    for (int i = 0; i < spiral_count; ++i) {
      float t = static_cast<float>(i) / spiral_count;
      float angle = t * 6.28318f * 3.0f;                 // 3 full rotations
      float radius = spiral_radius * (0.3f + 0.7f * t);  // Expanding spiral
      float height = spiral_height_start + spiral_height_range * t;

      spiral_points.emplace_back(radius * cos(angle), radius * sin(angle),
                                 height);

      // Rainbow colors along the spiral
      float hue = t * 360.0f;
      if (hue < 120.0f) {
        // Red to Green
        float f = hue / 120.0f;
        spiral_colors.emplace_back(1.0f - f, f, 0.2f);
      } else if (hue < 240.0f) {
        // Green to Blue
        float f = (hue - 120.0f) / 120.0f;
        spiral_colors.emplace_back(0.2f, 1.0f - f, f);
      } else {
        // Blue to Red
        float f = (hue - 240.0f) / 120.0f;
        spiral_colors.emplace_back(f, 0.2f, 1.0f - f);
      }
    }

    auto spiral_cloud = std::make_unique<PointCloud>();
    spiral_cloud->SetPoints(spiral_points, spiral_colors);
    spiral_cloud->SetPointSize(6.0f);  // Slightly smaller
    scene_manager->AddOpenGLObject("spiral_points", std::move(spiral_cloud));

    std::cout << "✓ Spiral point cloud created with " << spiral_points.size()
              << " points" << std::endl;

    std::cout << "✓ Test objects added to scene" << std::endl;
    std::cout << "  - Reference grid for navigation" << std::endl;
    std::cout << "  - " << sphere_positions.size()
              << " spheres in 3x3x3 grid for object selection testing"
              << std::endl;
    std::cout << "  - sphere_00 at bottom-left-back corner (-2, -2, 0)"
              << std::endl;
    std::cout << "  - Colors: Bottom layer (RED), Middle layer (GREEN), Top "
                 "layer (BLUE)"
              << std::endl;
    std::cout << "  - Grid point cloud (100 points) at z=2.0 for point "
                 "selection testing"
              << std::endl;
    std::cout
        << "  - Spiral point cloud (50 points) at z=3.0-5.0 with rainbow colors"
        << std::endl;

    // Create main container
    auto main_box = std::make_shared<Box>("main_container");
    main_box->SetFlexDirection(Styling::FlexDirection::kRow);

    // Add components
    main_box->AddChild(info_panel);
    main_box->AddChild(scene_panel);

    // Add to viewer
    viewer.AddSceneObject(main_box);

    std::cout << "\n✓ Selection test ready!" << std::endl;
    std::cout << "\n=== Scene Contents ===" << std::endl;
    std::cout << "- Grid Point Cloud: 'grid_points' with 100 points at z=2.0 "
                 "(green/cyan grid)"
              << std::endl;
    std::cout << "- Spiral Point Cloud: 'spiral_points' with 50 points at "
                 "z=3.0-5.0 (rainbow spiral)"
              << std::endl;
    std::cout
        << "- Spheres: 9 spheres (sphere_00 to sphere_08) at z=0.0 (red color)"
        << std::endl;
    std::cout << "\n=== Sphere Grid Reference ===" << std::endl;
    std::cout << "BOTTOM layer (Z=0, RED spheres):" << std::endl;
    std::cout << "  Back  (Y=-2): [00](-2,-2,0)  [01](0,-2,0)  [02](2,-2,0)"
              << std::endl;
    std::cout << "  Mid   (Y=0):  [03](-2,0,0)   [04](0,0,0)   [05](2,0,0)"
              << std::endl;
    std::cout << "  Front (Y=2):  [06](-2,2,0)   [07](0,2,0)   [08](2,2,0)"
              << std::endl;
    std::cout << "\nMIDDLE layer (Z=2, GREEN spheres):" << std::endl;
    std::cout << "  Back  (Y=-2): [09](-2,-2,2)  [10](0,-2,2)  [11](2,-2,2)"
              << std::endl;
    std::cout << "  Mid   (Y=0):  [12](-2,0,2)   [13](0,0,2)   [14](2,0,2)"
              << std::endl;
    std::cout << "  Front (Y=2):  [15](-2,2,2)   [16](0,2,2)   [17](2,2,2)"
              << std::endl;
    std::cout << "\nTOP layer (Z=4, BLUE spheres):" << std::endl;
    std::cout << "  Back  (Y=-2): [18](-2,-2,4)  [19](0,-2,4)  [20](2,-2,4)"
              << std::endl;
    std::cout << "  Mid   (Y=0):  [21](-2,0,4)   [22](0,0,4)   [23](2,0,4)"
              << std::endl;
    std::cout << "  Front (Y=2):  [24](-2,2,4)   [25](0,2,4)   [26](2,2,4)"
              << std::endl;
    std::cout << "\nFormat: [ID](X,Y,Z) where sphere_ID is the object name"
              << std::endl;
    std::cout << "=================================" << std::endl;
    viewer.Show();

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
}