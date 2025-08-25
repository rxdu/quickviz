/**
 * @file test_point_picking.cpp
 * @date 2025-08-25
 * @brief Test application for point cloud picking and selection
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <random>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/text3d.hpp"
#include "gldraw/camera_controller.hpp"
#include "gldraw/gl_view.hpp"

#include "visualization/selection/point_cloud_selector.hpp"

using namespace quickviz;
using namespace quickviz::visualization;

class PointPickingView : public GlView {
 public:
  PointPickingView() : GlView("Point Cloud Picking Test", 1280, 720) {
    SetupScene();
  }

  void SetupScene() override {
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
    
    // Add reference grid
    auto grid = std::make_shared<Grid>(1.0f, 10, glm::vec3(0.5f, 0.5f, 0.5f));
    scene_manager_->AddOpenGLObject("grid", grid);
    
    // Add coordinate frame
    auto frame = std::make_shared<CoordinateFrame>(2.0f);
    scene_manager_->AddOpenGLObject("frame", frame);
    
    // Add pick indicator sphere (initially hidden)
    pick_indicator_ = std::make_shared<Sphere>(0.05f);
    pick_indicator_->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    pick_indicator_->SetPosition(glm::vec3(0));
    pick_indicator_->SetVisible(false);
    scene_manager_->AddOpenGLObject("pick_indicator", pick_indicator_);
    
    // Add selection info text
    info_text_ = std::make_shared<Text3d>("No selection", 0.3f);
    info_text_->SetPosition(glm::vec3(-4.0f, 3.0f, 0.0f));
    info_text_->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
    scene_manager_->AddOpenGLObject("info_text", info_text_);
    
    // Setup camera
    camera_controller_->SetCameraPosition(glm::vec3(5.0f, 5.0f, 5.0f));
    camera_controller_->SetCameraTarget(glm::vec3(0.0f, 0.0f, 0.0f));
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
  }

  void OnMouseButton(int button, int action, int mods) override {
    GlView::OnMouseButton(button, action, mods);
    
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
      if (!ImGui::GetIO().WantCaptureMouse) {
        // Get mouse ray from camera
        auto ray = GetMouseRay();
        
        if (ray.has_value()) {
          // Perform point picking
          PerformPointPicking(ray.value(), mods);
        }
      }
    }
    
    if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
      if (!ImGui::GetIO().WantCaptureMouse) {
        // Clear selection on right click
        selector_->ClearSelection();
        selector_->ApplySelectionVisualization();
        pick_indicator_->SetVisible(false);
        UpdateInfoText();
      }
    }
  }

  void OnKey(int key, int scancode, int action, int mods) override {
    GlView::OnKey(key, scancode, action, mods);
    
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
      switch (key) {
        case GLFW_KEY_S:
          // Sphere selection around last picked point
          if (last_pick_position_.has_value()) {
            auto indices = selector_->SelectInSphere(last_pick_position_.value(), 0.5f);
            selector_->UpdateSelection(indices, 
              (mods & GLFW_MOD_SHIFT) ? SelectionMode::kAdditive : SelectionMode::kSingle);
            selector_->ApplySelectionVisualization();
            UpdateInfoText();
          }
          break;
          
        case GLFW_KEY_B:
          // Box selection around last picked point
          if (last_pick_position_.has_value()) {
            glm::vec3 center = last_pick_position_.value();
            glm::vec3 half_size(0.5f);
            auto indices = selector_->SelectInBox(center - half_size, center + half_size);
            selector_->UpdateSelection(indices,
              (mods & GLFW_MOD_SHIFT) ? SelectionMode::kAdditive : SelectionMode::kSingle);
            selector_->ApplySelectionVisualization();
            UpdateInfoText();
          }
          break;
          
        case GLFW_KEY_C:
          // Clear selection
          selector_->ClearSelection();
          selector_->ApplySelectionVisualization();
          pick_indicator_->SetVisible(false);
          UpdateInfoText();
          break;
          
        case GLFW_KEY_SPACE:
          // Print selection statistics
          PrintSelectionStats();
          break;
      }
    }
  }

  std::optional<Ray> GetMouseRay() {
    // Get normalized device coordinates
    double xpos, ypos;
    glfwGetCursorPos(window_, &xpos, &ypos);
    
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    
    float x = (2.0f * xpos) / width - 1.0f;
    float y = 1.0f - (2.0f * ypos) / height;
    
    // Get view and projection matrices
    glm::mat4 view = camera_controller_->GetViewMatrix();
    glm::mat4 projection = glm::perspective(
      glm::radians(45.0f),
      static_cast<float>(width) / static_cast<float>(height),
      0.1f, 1000.0f
    );
    
    // Unproject to get ray
    glm::mat4 inv_vp = glm::inverse(projection * view);
    
    glm::vec4 ray_start_ndc(x, y, -1.0f, 1.0f);
    glm::vec4 ray_end_ndc(x, y, 1.0f, 1.0f);
    
    glm::vec4 ray_start_world = inv_vp * ray_start_ndc;
    ray_start_world /= ray_start_world.w;
    
    glm::vec4 ray_end_world = inv_vp * ray_end_ndc;
    ray_end_world /= ray_end_world.w;
    
    glm::vec3 origin(ray_start_world);
    glm::vec3 direction = glm::normalize(glm::vec3(ray_end_world) - origin);
    
    return Ray(origin, direction);
  }

  void PerformPointPicking(const Ray& ray, int mods) {
    // Pick point with ray
    auto result = selector_->PickPoint(ray, 0.2f); // 0.2 units tolerance
    
    if (result.has_value()) {
      // Update pick indicator
      pick_indicator_->SetPosition(result->point);
      pick_indicator_->SetVisible(true);
      last_pick_position_ = result->point;
      
      // Update selection based on modifier keys
      SelectionMode mode = SelectionMode::kSingle;
      if (mods & GLFW_MOD_SHIFT) {
        mode = SelectionMode::kAdditive;
      } else if (mods & GLFW_MOD_CONTROL) {
        mode = SelectionMode::kSubtractive;
      } else if (mods & GLFW_MOD_ALT) {
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
    }
  }

  void RenderImGui() override {
    ImGui::Begin("Point Picking Controls");
    
    ImGui::Text("Mouse Controls:");
    ImGui::BulletText("Left Click: Pick point");
    ImGui::BulletText("Shift + Left Click: Add to selection");
    ImGui::BulletText("Ctrl + Left Click: Remove from selection");
    ImGui::BulletText("Alt + Left Click: Toggle selection");
    ImGui::BulletText("Right Click: Clear selection");
    
    ImGui::Separator();
    ImGui::Text("Keyboard Controls:");
    ImGui::BulletText("S: Sphere selection (0.5 radius)");
    ImGui::BulletText("B: Box selection (1x1x1)");
    ImGui::BulletText("C: Clear selection");
    ImGui::BulletText("Space: Print statistics");
    
    ImGui::Separator();
    size_t count = selector_->GetSelectionCount();
    ImGui::Text("Selected: %zu points", count);
    
    if (count > 0) {
      glm::vec3 centroid = selector_->GetSelectionCentroid();
      ImGui::Text("Centroid: (%.2f, %.2f, %.2f)", centroid.x, centroid.y, centroid.z);
    }
    
    ImGui::End();
  }

 private:
  std::shared_ptr<PointCloud> point_cloud_;
  std::unique_ptr<PointCloudSelector> selector_;
  std::shared_ptr<Sphere> pick_indicator_;
  std::shared_ptr<Text3d> info_text_;
  std::optional<glm::vec3> last_pick_position_;
};

int main(int argc, char* argv[]) {
  PointPickingView view;
  view.Run();
  return 0;
}