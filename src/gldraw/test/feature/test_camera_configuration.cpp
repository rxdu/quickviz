/*
 * test_camera_configuration.cpp
 *
 * Created on: Sept 2, 2025
 * Description: Test configurable camera controller parameters
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <glm/glm.hpp>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "gldraw/gl_scene_panel.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/camera_controller.hpp"
#include "imgui.h"

using namespace quickviz;

std::vector<glm::vec4> GenerateTestPointCloud() {
  std::vector<glm::vec4> test_points;
  for (int i = -2; i <= 2; ++i) {
    for (int j = -2; j <= 2; ++j) {
      for (int k = -2; k <= 2; ++k) {
        float intensity = (i + j + k + 6) / 12.0f;
        test_points.push_back(glm::vec4(i * 3.0f, j * 3.0f, k * 3.0f, intensity));
      }
    }
  }
  return test_points;
}

class CameraConfigPanel : public Panel {
public:
  CameraConfigPanel(const std::string& name, std::shared_ptr<GlScenePanel> scene_panel)
      : Panel(name), scene_panel_(scene_panel) {
    
    configs_ = {
      {"Default", CameraControllerConfig::Default()},
      {"Large Scale", CameraControllerConfig::LargeScale()},
      {"Precision", CameraControllerConfig::Precision()},
    };
    
    current_config_index_ = 0;
    ApplyCurrentConfig();
  }

  void Draw() override {
    if (ImGui::Begin(name_.c_str())) {
      ImGui::Text("Camera Controller Configuration");
      ImGui::Separator();
      
      // Configuration selector
      if (ImGui::BeginCombo("Configuration Preset", configs_[current_config_index_].first.c_str())) {
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
      ImGui::Text("Configuration Parameters:");
      
      // Show current configuration values
      const auto& config = configs_[current_config_index_].second;
      
      ImGui::Text("Pan Sensitivity: %.4f", config.pan_sensitivity);
      ImGui::Text("Rotation Sensitivity: %.2f", config.orbit_rotation_sensitivity);
      ImGui::Text("Distance Scale: %.1f", config.distance_scale_factor);
      ImGui::Text("Height Scale: %.1f", config.height_scale_factor);
      ImGui::Text("Min Orbit Distance: %.1f", config.min_orbit_distance);
      ImGui::Text("Min Height: %.1f", config.min_height);
      ImGui::Text("Orbit Zoom Speed: %.1f", config.orbit_zoom_speed);
      ImGui::Text("TopDown Zoom Speed: %.1f", config.topdown_zoom_speed);
      ImGui::Text("Initial Orbit Distance: %.1f", config.initial_orbit_distance);
      
      ImGui::Separator();
      ImGui::Text("Configuration Descriptions:");
      ImGui::BulletText("Default: Standard 3D modeling sensitivity");
      ImGui::BulletText("Large Scale: For big scenes (higher sensitivity)");
      ImGui::BulletText("Precision: For fine work (lower sensitivity)");
      
      ImGui::Separator();
      ImGui::Text("Test different configurations with mouse controls!");
    }
    ImGui::End();
  }

private:
  void ApplyCurrentConfig() {
    const auto& config = configs_[current_config_index_].second;
    auto* camera_controller = scene_panel_->GetCameraController();
    if (camera_controller) {
      camera_controller->SetConfig(config);
    }
  }
  
  std::shared_ptr<GlScenePanel> scene_panel_;
  std::vector<std::pair<std::string, CameraControllerConfig>> configs_;
  size_t current_config_index_;
};

int main() {
  std::cout << "Testing Configurable Camera Controller Parameters" << std::endl;
  std::cout << "===============================================" << std::endl;

  Viewer viewer;
  auto main_box = std::make_shared<Box>("main_box");
  main_box->SetFlexDirection(Styling::FlexDirection::kRow);

  // Create the main 3D scene panel
  auto scene_panel = std::make_shared<GlScenePanel>("3D Scene");
  scene_panel->SetAutoLayout(true);
  scene_panel->SetFlexGrow(2.0f);
  scene_panel->SetFlexShrink(0.0f);
  
  // Add content to the scene
  auto point_cloud = std::make_unique<PointCloud>();
  point_cloud->SetPoints(GenerateTestPointCloud(), 
                        PointCloud::ColorMode::kScalarField);
  point_cloud->SetScalarRange(0.0f, 1.0f);
  point_cloud->SetPointSize(6.0f);
  point_cloud->SetRenderMode(PointMode::kPoint);

  auto grid = std::make_unique<Grid>(15.0f, 1.0f, glm::vec3(0.3f, 0.3f, 0.3f));

  scene_panel->AddOpenGLObject("point_cloud", std::move(point_cloud));
  scene_panel->AddOpenGLObject("grid", std::move(grid));

  // Create the configuration panel
  auto config_panel = std::make_shared<CameraConfigPanel>("Camera Configuration", scene_panel);
  config_panel->SetAutoLayout(true);
  config_panel->SetFlexGrow(1.0f);
  config_panel->SetFlexShrink(0.0f);

  main_box->AddChild(scene_panel);
  main_box->AddChild(config_panel);
  
  viewer.AddSceneObject(main_box);

  std::cout << "\n=== Instructions ===" << std::endl;
  std::cout << "1. Select different configuration presets from the dropdown" << std::endl;
  std::cout << "2. Notice how camera sensitivity changes with each preset" << std::endl;
  std::cout << "3. Default: Standard modeling software feel" << std::endl;
  std::cout << "4. Large Scale: Higher sensitivity for large scenes" << std::endl;
  std::cout << "5. Precision: Lower sensitivity for detailed work" << std::endl;

  viewer.Show();
  return 0;
}