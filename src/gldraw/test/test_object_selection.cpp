/*
 * test_object_selection.cpp
 *
 * Created on: Jan 2025
 * Description: Test object selection functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <cmath>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "imview/panel.hpp"

#include "gldraw/scene_view_panel.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/grid.hpp"

using namespace quickviz;

// Simple info panel to display selection status
class SelectionInfoPanel : public Panel {
 public:
  SelectionInfoPanel(const std::string& title) : Panel(title) {}
  
  void SetSelectedObject(const std::string& name) {
    selected_object_ = name;
  }
  
  void Draw() override {
    Begin();
    
    ImGui::Text("Object Selection Demo");
    ImGui::Separator();
    
    if (selected_object_.empty()) {
      ImGui::Text("No object selected");
      ImGui::Text("Click on a sphere to select it");
    } else {
      ImGui::Text("Selected: %s", selected_object_.c_str());
    }
    
    ImGui::Separator();
    ImGui::Text("Controls:");
    ImGui::BulletText("Left Click: Select object");
    ImGui::BulletText("Right Click: Clear selection");
    ImGui::BulletText("ESC: Exit");
    
    End();
  }
  
 private:
  std::string selected_object_;
};

// Extended scene panel with object selection handling
class SelectableSceneManager : public SceneViewPanel {
 public:
  SelectableSceneManager(const std::string& name, SelectionInfoPanel* info_panel) 
      : SceneViewPanel(name), info_panel_(info_panel) {
    // Set up object selection callback
    SetObjectSelectionCallback([this](const std::string& name) {
      if (info_panel_) {
        info_panel_->SetSelectedObject(name);
      }
      if (!name.empty()) {
        std::cout << "Selected object: " << name << std::endl;
      } else {
        std::cout << "Selection cleared" << std::endl;
      }
    });
  }
  
  void Draw() override {
    Begin();
    
    // Handle mouse input for object selection
    if (ImGui::IsWindowHovered() && !ImGui::IsAnyItemHovered()) {
      ImGuiIO& io = ImGui::GetIO();
      ImVec2 content_size = ImGui::GetContentRegionAvail();
      ImVec2 window_pos = ImGui::GetWindowPos();
      ImVec2 window_content_min = ImGui::GetWindowContentRegionMin();
      
      float local_x = io.MousePos.x - window_pos.x - window_content_min.x;
      float local_y = io.MousePos.y - window_pos.y - window_content_min.y;
      
      bool mouse_in_viewport = (local_x >= 0 && local_x <= content_size.x &&
                                local_y >= 0 && local_y <= content_size.y);
      
      if (mouse_in_viewport) {
        // Left click to select
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
          SelectObjectAt(local_x, local_y);
        }
        
        // Right click to clear selection
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
          ClearObjectSelection();
        }
      }
    }
    
    RenderInsideWindow();
    End();
  }
  
 private:
  SelectionInfoPanel* info_panel_;
};

int main() {
  std::cout << "=== Object Selection Test ===" << std::endl;
  std::cout << "Click on spheres to select them" << std::endl;
  
  // Create viewer
  Viewer viewer("Object Selection Demo");
  
  // Create main container
  auto main_box = std::make_shared<Box>("main_container");
  main_box->SetFlexDirection(Styling::FlexDirection::kRow);
  main_box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  main_box->SetAlignItems(Styling::AlignItems::kStretch);
  
  // Create info panel
  auto info_panel = std::make_shared<SelectionInfoPanel>("Selection Info");
  info_panel->SetAutoLayout(true);
  info_panel->SetFlexBasis(250.0f);
  info_panel->SetFlexGrow(0.0f);
  info_panel->SetFlexShrink(0.0f);
  
  // Create scene panel with selection support
  auto scene_manager = std::make_shared<SelectableSceneManager>("3D Scene", info_panel.get());
  scene_manager->SetAutoLayout(true);
  scene_manager->SetFlexGrow(1.0f);
  scene_manager->SetBackgroundColor(0.1f, 0.1f, 0.2f, 1.0f);
  
  // Add reference grid
  auto grid = std::make_unique<Grid>(1.0f, 20.0f, glm::vec3(0.5f, 0.5f, 0.5f));
  scene_manager->AddOpenGLObject("grid", std::move(grid));
  
  // Create several spheres at different positions
  std::vector<glm::vec3> sphere_positions = {
    {0.0f, 0.0f, 1.0f},    // Center
    {3.0f, 0.0f, 1.0f},    // Right
    {-3.0f, 0.0f, 1.0f},   // Left
    {0.0f, 3.0f, 1.0f},    // Front
    {0.0f, -3.0f, 1.0f},   // Back
    {2.0f, 2.0f, 2.0f},    // Upper right
    {-2.0f, 2.0f, 2.0f},   // Upper left
    {2.0f, -2.0f, 0.5f},   // Lower right
    {-2.0f, -2.0f, 0.5f}   // Lower left
  };
  
  std::vector<glm::vec3> sphere_colors = {
    {0.9f, 0.3f, 0.3f},    // Red
    {0.3f, 0.9f, 0.3f},    // Green
    {0.3f, 0.3f, 0.9f},    // Blue
    {0.9f, 0.9f, 0.3f},    // Yellow
    {0.9f, 0.3f, 0.9f},    // Magenta
    {0.3f, 0.9f, 0.9f},    // Cyan
    {0.9f, 0.6f, 0.3f},    // Orange
    {0.6f, 0.3f, 0.9f},    // Purple
    {0.3f, 0.9f, 0.6f}     // Teal
  };
  
  // Add spheres to the scene
  for (size_t i = 0; i < sphere_positions.size(); ++i) {
    auto sphere = std::make_unique<Sphere>(sphere_positions[i], 0.5f);
    sphere->SetColor(sphere_colors[i]);
    sphere->SetRenderMode(Sphere::RenderMode::kSolid);
    
    std::string name = "sphere_" + std::to_string(i);
    scene_manager->AddOpenGLObject(name, std::move(sphere));
  }
  
  // Add components to container
  main_box->AddChild(info_panel);
  main_box->AddChild(scene_manager);
  
  // Add to viewer
  viewer.AddSceneObject(main_box);
  
  std::cout << "Visualization ready. Click on spheres to select them." << std::endl;
  viewer.Show();
  
  return 0;
}