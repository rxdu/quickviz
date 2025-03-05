/*
 * test_point_cloud.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description: Interactive point cloud visualization with camera controls
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <thread>
#include <random>

#include <opencv2/opencv.hpp>

#include "imview/viewer.hpp"
#include "imview/widget/gl_widget.hpp"
#include "imview/component/opengl/point_cloud.hpp"
#include "imview/component/opengl/grid.hpp"
#include "imview/component/opengl/camera.hpp"
#include "imview/component/opengl/camera_controller.hpp"

using namespace quickviz;

// Custom GL widget with camera controls
class InteractiveGLWidget : public GlWidget {
 public:
  InteractiveGLWidget(const std::string& name) : GlWidget(name) {
    camera_ = std::make_unique<Camera>();
    camera_controller_ = std::make_unique<CameraController>(
        *camera_, glm::vec3(0.0f, 0.0f, 3.0f), 0.0f, 0.0f);
      
    // Print camera position for debugging
    auto pos = camera_->GetPosition();
    std::cout << "Camera position: " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
  }

  void Draw() override {
    Begin();

    // Update view according to user input
    ImGuiIO& io = ImGui::GetIO();
    
    // Only process mouse delta when mouse position is within the widget
    if (ImGui::IsMousePosValid() && ImGui::IsWindowHovered()) {
      // Track mouse move delta only when the mouse left button is pressed
      if (ImGui::IsMouseDown(0)) { // Left mouse button
        camera_controller_->ProcessMouseMovement(io.MouseDelta.x, io.MouseDelta.y);
        std::cout << "Camera moved: " << io.MouseDelta.x << ", " << io.MouseDelta.y << std::endl;
      }

      // Track mouse wheel scroll
      if (io.MouseWheel != 0.0f) {
        camera_controller_->ProcessMouseScroll(io.MouseWheel);
        std::cout << "Camera zoomed: " << io.MouseWheel << std::endl;
      }
    }

    // Get view matrices from camera
    ImVec2 content_size = ImGui::GetContentRegionAvail();
    float aspect_ratio = static_cast<float>(content_size.x) / static_cast<float>(content_size.y);
    glm::mat4 projection = camera_->GetProjectionMatrix(aspect_ratio);
    glm::mat4 view = camera_->GetViewMatrix();
    
    // Update view matrices
    UpdateView(projection, view);

    // Draw the scene
    DrawOpenGLObject();

    // Display instructions
    ImGui::SetCursorPos(ImVec2(10, 10));
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), 
                      "Left-click + drag: Rotate camera");
    ImGui::SetCursorPos(ImVec2(10, 30));
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), 
                      "Mouse wheel: Zoom in/out");

    End();
  }

 private:
  std::unique_ptr<Camera> camera_;
  std::unique_ptr<CameraController> camera_controller_;
};

int main(int argc, char* argv[]) {
  Viewer viewer;

  // Create interactive GL widget with camera controls
  auto gl_widget = std::make_shared<InteractiveGLWidget>("Point Cloud Viewer");
  gl_widget->OnResize(1024, 768);
  gl_widget->SetPosition(0, 0);
  
  std::cout << "Generating random points..." << std::endl;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> dist(0.0f, 0.5f);  // Standard deviation of 0.5 instead of 1.0

  // Generate 3D points with a Gaussian distribution
  std::vector<glm::vec3> points;
  std::vector<glm::vec3> colors;

  // Generate 1000 random points
  for (int i = 0; i < 1000; ++i) {
    float x = dist(gen)*10;
    float y = dist(gen)*10;
    float z = dist(gen)*10;
    points.push_back(glm::vec3(x, y, z));
    
    // Use bright colors for better visibility
    // Map each point to a bright color based on its position
    colors.push_back(glm::vec3(
      fabs(x) + 0.5f,  // Brighter red (0.5-1.5 range)
      fabs(y) + 0.5f,  // Brighter green (0.5-1.5 range)
      fabs(z) + 0.5f   // Brighter blue (0.5-1.5 range)
    ));
  }

  std::cout << "Generated " << points.size() << " random points with Gaussian distribution" << std::endl;

  // Add a grid for reference
  // auto grid = std::make_unique<Grid>(10.0f, 1.0f);
  // gl_widget->AddOpenGLObject("grid", std::move(grid));
  
  // Create and configure point cloud with very large points
  auto point_cloud = std::make_unique<PointCloud>();
  point_cloud->SetPoints(points);
  point_cloud->SetColors(colors);  // Use direct colors
  point_cloud->SetPointSize(50.0f);  // Make points extremely large for visibility
  point_cloud->SetOpacity(1.0f);
  point_cloud->SetRenderMode(PointRenderMode::Points);

  // Add this line to print the point cloud data for debugging
  std::cout << "First point: " << points[0].x << ", " << points[0].y << ", " << points[0].z << std::endl;
  std::cout << "First color: " << colors[0].x << ", " << colors[0].y << ", " << colors[0].z << std::endl;

  gl_widget->AddOpenGLObject("point_cloud", std::move(point_cloud));

  viewer.AddSceneObject(gl_widget);
  std::cout << "Starting viewer with interactive camera controls..." << std::endl;
  std::cout << "- Left-click + drag to rotate the camera" << std::endl;
  std::cout << "- Use mouse wheel to zoom in/out" << std::endl;
  viewer.Show();

  return 0;
}