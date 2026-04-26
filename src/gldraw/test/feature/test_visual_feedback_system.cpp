/*
 * test_visual_feedback_system.cpp
 *
 * Created on: Sept 2, 2025
 * Description: Test the visual feedback system architecture and integration
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <glm/glm.hpp>

#include "viewer/viewer.hpp"
#include "viewer/box.hpp"
#include "gldraw/gl_scene_panel.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/triangle.hpp"
#include "gldraw/feedback/visual_feedback_system.hpp"
#include "imgui.h"

using namespace quickviz;

std::vector<glm::vec4> GenerateTestPointCloud() {
  std::vector<glm::vec4> test_points;
  for (int i = -3; i <= 3; ++i) {
    for (int j = -3; j <= 3; ++j) {
      for (int k = -3; k <= 3; ++k) {
        float intensity = (i + j + k + 9) / 18.0f;
        test_points.push_back(glm::vec4(i * 2.0f, j * 2.0f, k * 2.0f, intensity));
      }
    }
  }
  return test_points;
}

class FeedbackTestPanel : public Panel {
public:
  FeedbackTestPanel(const std::string& name, std::shared_ptr<GlScenePanel> scene_panel)
      : Panel(name), scene_panel_(scene_panel), feedback_system_(nullptr) {
    
    if (scene_panel_) {
      feedback_system_ = scene_panel_->GetFeedbackSystem();
    }
    
    // Initialize themes
    themes_ = {
      {"Default", FeedbackTheme::Default()},
      {"High Contrast", FeedbackTheme::HighContrast()},
      {"Subtle", FeedbackTheme::Subtle()},
      {"CAD Style", FeedbackTheme::CADStyle()},
    };
    
    current_theme_index_ = 0;
  }

  void Draw() override {
    if (ImGui::Begin(name_.c_str())) {
      ImGui::Text("Visual Feedback System Test");
      ImGui::Separator();
      
      if (!feedback_system_) {
        ImGui::TextColored(ImVec4(1,0,0,1), "Error: Feedback system not available");
        ImGui::End();
        return;
      }
      
      // Theme selection
      if (ImGui::BeginCombo("Theme", themes_[current_theme_index_].first.c_str())) {
        for (size_t i = 0; i < themes_.size(); ++i) {
          bool is_selected = (current_theme_index_ == i);
          if (ImGui::Selectable(themes_[i].first.c_str(), is_selected)) {
            current_theme_index_ = i;
            feedback_system_->SetTheme(themes_[i].second);
          }
          if (is_selected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      
      ImGui::Separator();
      ImGui::Text("Feedback Controls:");
      
      // Enable/disable feedback system
      bool enabled = feedback_system_->IsEnabled();
      if (ImGui::Checkbox("Enable Feedback System", &enabled)) {
        feedback_system_->SetEnabled(enabled);
      }
      
      ImGui::Separator();
      ImGui::Text("Object Feedback Test:");
      
      // Test buttons for different feedback types
      if (ImGui::Button("Show Point Cloud Hover")) {
        feedback_system_->ShowFeedback("point_cloud", FeedbackType::kHover);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Point Cloud Hover")) {
        feedback_system_->RemoveFeedback("point_cloud", FeedbackType::kHover);
      }
      
      if (ImGui::Button("Show Point Cloud Selection")) {
        feedback_system_->ShowFeedback("point_cloud", FeedbackType::kSelection);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Point Cloud Selection")) {
        feedback_system_->RemoveFeedback("point_cloud", FeedbackType::kSelection);
      }
      
      if (ImGui::Button("Show Triangle Hover")) {
        feedback_system_->ShowFeedback("triangle", FeedbackType::kHover);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Triangle Hover")) {
        feedback_system_->RemoveFeedback("triangle", FeedbackType::kHover);
      }
      
      if (ImGui::Button("Show Triangle Selection")) {
        feedback_system_->ShowFeedback("triangle", FeedbackType::kSelection);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Triangle Selection")) {
        feedback_system_->RemoveFeedback("triangle", FeedbackType::kSelection);
      }
      
      ImGui::Separator();
      if (ImGui::Button("Clear All Feedback")) {
        feedback_system_->ClearAllFeedback();
      }
      
      ImGui::Separator();
      ImGui::Text("Feedback System Status:");
      ImGui::Text("Active feedback count: %zu", feedback_system_->GetActiveFeedbackCount());
      ImGui::Text("System enabled: %s", feedback_system_->IsEnabled() ? "Yes" : "No");
      
      // Show objects with different feedback types
      auto hover_objects = feedback_system_->GetObjectsWithFeedback(FeedbackType::kHover);
      ImGui::Text("Objects with hover feedback: %zu", hover_objects.size());
      for (const auto& obj : hover_objects) {
        ImGui::BulletText("%s", obj.c_str());
      }
      
      auto selection_objects = feedback_system_->GetObjectsWithFeedback(FeedbackType::kSelection);
      ImGui::Text("Objects with selection feedback: %zu", selection_objects.size());
      for (const auto& obj : selection_objects) {
        ImGui::BulletText("%s", obj.c_str());
      }
      
      ImGui::Separator();
      ImGui::Text("Instructions:");
      ImGui::BulletText("Use buttons above to test different feedback types");
      ImGui::BulletText("Try different themes to see visual changes");
      ImGui::BulletText("Note: Point cloud feedback uses LayerManager system");
      ImGui::BulletText("Note: Triangle feedback will use overlay system (when implemented)");
    }
    ImGui::End();
  }

private:
  std::shared_ptr<GlScenePanel> scene_panel_;
  VisualFeedbackSystem* feedback_system_;
  
  std::vector<std::pair<std::string, FeedbackTheme>> themes_;
  size_t current_theme_index_;
};

int main() {
  std::cout << "Testing Visual Feedback System Architecture" << std::endl;
  std::cout << "===========================================" << std::endl;

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
  point_cloud->SetPointSize(4.0f);
  point_cloud->SetRenderMode(PointMode::kPoint);

  auto triangle = std::make_unique<Triangle>(2.0f, glm::vec3(1.0f, 0.5f, 0.0f));
  
  auto grid = std::make_unique<Grid>(20.0f, 2.0f, glm::vec3(0.3f, 0.3f, 0.3f));

  scene_panel->AddOpenGLObject("point_cloud", std::move(point_cloud));
  scene_panel->AddOpenGLObject("triangle", std::move(triangle));
  scene_panel->AddOpenGLObject("grid", std::move(grid));

  // Create the feedback test panel
  auto test_panel = std::make_shared<FeedbackTestPanel>("Feedback System Test", scene_panel);
  test_panel->SetAutoLayout(true);
  test_panel->SetFlexGrow(1.0f);
  test_panel->SetFlexShrink(0.0f);

  main_box->AddChild(scene_panel);
  main_box->AddChild(test_panel);
  
  viewer.AddSceneObject(main_box);

  std::cout << "\n=== Visual Feedback System Test ===" << std::endl;
  std::cout << "1. Use the control panel to test different feedback types" << std::endl;
  std::cout << "2. Try different visual themes" << std::endl;
  std::cout << "3. Observe the coordination layer working with specialized handlers" << std::endl;
  std::cout << "4. Point cloud feedback will use existing LayerManager" << std::endl;
  std::cout << "5. Triangle feedback will demonstrate object handler (placeholder for now)" << std::endl;

  viewer.Show();
  return 0;
}