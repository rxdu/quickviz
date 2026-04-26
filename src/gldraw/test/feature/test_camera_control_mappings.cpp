/*
 * test_switchable_camera_controls.cpp
 *
 * Created on: Sept 2, 2025
 * Description: Test single scene panel with switchable camera control configurations
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <vector>
#include <string>
#include <glm/glm.hpp>

#include "viewer/viewer.hpp"
#include "viewer/box.hpp"
#include "gldraw/gl_scene_panel.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/scene_input_handler.hpp"
#include "gldraw/camera_control_config.hpp"
#include "imgui.h"

using namespace quickviz;

std::vector<glm::vec4> GenerateTestPointCloud() {
  std::vector<glm::vec4> test_points;
  for (int i = -4; i <= 4; ++i) {
    for (int j = -4; j <= 4; ++j) {
      for (int k = -4; k <= 4; ++k) {
        float intensity = (i + j + k + 12) / 24.0f;
        test_points.push_back(glm::vec4(i * 1.5f, j * 1.5f, k * 1.5f, intensity));
      }
    }
  }
  return test_points;
}

class CameraControlPanel : public Panel {
public:
  CameraControlPanel(const std::string& name, std::shared_ptr<GlScenePanel> scene_panel)
      : Panel(name), scene_panel_(scene_panel), current_config_index_(0) {
    
    // Initialize available configurations
    configs_ = {
      {"Modeling Software (Blender/Maya)", CameraControlConfig::ModelingSoftware()},
      {"FPS Style", CameraControlConfig::FPSStyle()},
      {"Web Viewer", CameraControlConfig::WebViewer()},
      {"CAD Style (SolidWorks)", CameraControlConfig::CADStyle()},
      {"Scientific Visualization", CameraControlConfig::Scientific()},
      {"Single Button (Tablet)", CameraControlConfig::SingleButton()}
    };
    
    // Set initial configuration
    ApplyCurrentConfig();
  }

  void Draw() override {
    if (ImGui::Begin(name_.c_str())) {
      ImGui::Text("Camera Control Configuration");
      ImGui::Separator();
      
      // Configuration selector
      if (ImGui::BeginCombo("Control Scheme", configs_[current_config_index_].first.c_str())) {
        for (size_t i = 0; i < configs_.size(); ++i) {
          bool is_selected = (current_config_index_ == i);
          if (ImGui::Selectable(configs_[i].first.c_str(), is_selected)) {
            current_config_index_ = i;
            ApplyCurrentConfig();
          }
          if (is_selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      
      ImGui::Separator();
      
      // Show current configuration details
      const auto& current_config = configs_[current_config_index_].second;
      ImGui::Text("Current Controls:");
      
      // Show button mappings
      if (current_config.enable_orbit) {
        ImGui::Text("Orbit: %s", GetMouseButtonName(current_config.orbit_button).c_str());
      }
      if (current_config.enable_pan) {
        ImGui::Text("Pan: %s", GetMouseButtonName(current_config.pan_button).c_str());
      }
      if (current_config.enable_selection) {
        ImGui::Text("Select: %s", GetMouseButtonName(current_config.selection_button).c_str());
      }
      if (current_config.enable_wheel_zoom) {
        ImGui::Text("Zoom: Mouse Wheel");
      }
      
      ImGui::Separator();
      
      // Sensitivity controls
      ImGui::Text("Sensitivity Settings:");
      
      float orbit_sens = current_config.orbit_sensitivity;
      if (ImGui::SliderFloat("Orbit Sensitivity", &orbit_sens, 0.1f, 3.0f)) {
        auto modified_config = current_config;
        modified_config.orbit_sensitivity = orbit_sens;
        scene_panel_->GetSceneInputHandler()->SetCameraControlConfig(modified_config);
        configs_[current_config_index_].second = modified_config;
      }
      
      float pan_sens = current_config.pan_sensitivity;
      if (ImGui::SliderFloat("Pan Sensitivity", &pan_sens, 0.1f, 3.0f)) {
        auto modified_config = current_config;
        modified_config.pan_sensitivity = pan_sens;
        scene_panel_->GetSceneInputHandler()->SetCameraControlConfig(modified_config);
        configs_[current_config_index_].second = modified_config;
      }
      
      float zoom_sens = current_config.zoom_sensitivity;
      if (ImGui::SliderFloat("Zoom Sensitivity", &zoom_sens, 0.1f, 3.0f)) {
        auto modified_config = current_config;
        modified_config.zoom_sensitivity = zoom_sens;
        scene_panel_->GetSceneInputHandler()->SetCameraControlConfig(modified_config);
        configs_[current_config_index_].second = modified_config;
      }
      
      ImGui::Separator();
      
      // Enable/disable toggles
      ImGui::Text("Enable/Disable:");
      
      bool orbit_enabled = current_config.enable_orbit;
      if (ImGui::Checkbox("Enable Orbit", &orbit_enabled)) {
        auto modified_config = current_config;
        modified_config.enable_orbit = orbit_enabled;
        scene_panel_->GetSceneInputHandler()->SetCameraControlConfig(modified_config);
        configs_[current_config_index_].second = modified_config;
      }
      
      bool selection_enabled = current_config.enable_selection;
      if (ImGui::Checkbox("Enable Selection", &selection_enabled)) {
        auto modified_config = current_config;
        modified_config.enable_selection = selection_enabled;
        scene_panel_->GetSceneInputHandler()->SetCameraControlConfig(modified_config);
        scene_panel_->GetSceneInputHandler()->SetSelectionEnabled(selection_enabled);
        configs_[current_config_index_].second = modified_config;
      }
      
      ImGui::Separator();
      ImGui::Text("Try different mouse interactions in the 3D view!");
    }
    ImGui::End();
  }

private:
  void ApplyCurrentConfig() {
    const auto& config = configs_[current_config_index_].second;
    scene_panel_->GetSceneInputHandler()->SetCameraControlConfig(config);
    scene_panel_->GetSceneInputHandler()->SetSelectionEnabled(config.enable_selection);
  }
  
  std::string GetMouseButtonName(MouseButton button) {
    switch (button) {
      case MouseButton::kLeft: return "Left Mouse";
      case MouseButton::kRight: return "Right Mouse";
      case MouseButton::kMiddle: return "Middle Mouse";
      case MouseButton::kNone: return "None";
      default: return "Unknown";
    }
  }

  std::shared_ptr<GlScenePanel> scene_panel_;
  std::vector<std::pair<std::string, CameraControlConfig>> configs_;
  size_t current_config_index_;
};

int main() {
  std::cout << "Testing Switchable Camera Control Configurations" << std::endl;
  std::cout << "===============================================" << std::endl;

  Viewer viewer;
  auto main_box = std::make_shared<Box>("main_box");
  main_box->SetFlexDirection(Styling::FlexDirection::kRow);

  // Create the main 3D scene panel
  auto scene_panel = std::make_shared<GlScenePanel>("3D Scene");
  scene_panel->SetAutoLayout(true);
  scene_panel->SetFlexGrow(2.0f); // Take up most of the space
  scene_panel->SetFlexShrink(0.0f);
  
  // Add content to the scene
  auto point_cloud = std::make_unique<PointCloud>();
  point_cloud->SetPoints(GenerateTestPointCloud(), 
                        PointCloud::ColorMode::kScalarField);
  point_cloud->SetScalarRange(0.0f, 1.0f);
  point_cloud->SetPointSize(4.0f);
  point_cloud->SetRenderMode(PointMode::kPoint);

  auto grid = std::make_unique<Grid>(12.0f, 1.0f, glm::vec3(0.4f, 0.4f, 0.4f));

  scene_panel->AddOpenGLObject("point_cloud", std::move(point_cloud));
  scene_panel->AddOpenGLObject("grid", std::move(grid));

  // Create the control panel
  auto control_panel = std::make_shared<CameraControlPanel>("Camera Controls", scene_panel);
  control_panel->SetAutoLayout(true);
  control_panel->SetFlexGrow(1.0f); // Smaller control panel
  control_panel->SetFlexShrink(0.0f);

  main_box->AddChild(scene_panel);
  main_box->AddChild(control_panel);
  
  viewer.AddSceneObject(main_box);

  std::cout << "\n=== Instructions ===" << std::endl;
  std::cout << "1. Use the dropdown in the control panel to switch between different camera control schemes" << std::endl;
  std::cout << "2. Adjust sensitivity settings with the sliders" << std::endl;
  std::cout << "3. Toggle features on/off with checkboxes" << std::endl;
  std::cout << "4. Try the different mouse interactions in the 3D scene" << std::endl;
  std::cout << "\nEach control scheme has different mouse button mappings!" << std::endl;

  viewer.Show();
  return 0;
}