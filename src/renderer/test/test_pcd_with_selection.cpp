/*
 * test_pcd_with_selection.cpp
 *
 * Created on: Dec 2024
 * Description: Load and visualize PCD files with interactive selection tools
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <vector>
#include <limits>
#include <memory>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "imview/box.hpp"
#include "imview/viewer.hpp"

#include "renderer/gl_scene_manager.hpp"
#include "renderer/renderable/grid.hpp"
#include "renderer/renderable/point_cloud.hpp"
#include "renderer/selection/selection_tools.hpp"

using namespace quickviz;

// Selection control panel for PCD viewer
class PCDSelectionPanel : public Panel {
 public:
  PCDSelectionPanel() : Panel("Selection Tools") {
    SetAutoLayout(true);
  }
  
  void Draw() override {
    ImGui::Text("PCD Selection Tools");
    ImGui::Separator();
    
    // File info
    if (!filename_.empty()) {
      ImGui::Text("File: %s", filename_.c_str());
      ImGui::Text("Points: %zu", total_points_);
      ImGui::Separator();
    }
    
    // Tool selection
    ImGui::Text("Selection Tool:");
    ImGui::RadioButton("None", &current_tool_, 0);
    ImGui::RadioButton("Point Pick", &current_tool_, 1);
    ImGui::RadioButton("Rectangle", &current_tool_, 2);
    ImGui::RadioButton("Lasso", &current_tool_, 3);
    ImGui::RadioButton("Radius", &current_tool_, 4);
    
    ImGui::Separator();
    
    // Selection mode
    ImGui::Text("Selection Mode:");
    ImGui::RadioButton("Replace", &selection_mode_, 0);
    ImGui::RadioButton("Add (Shift)", &selection_mode_, 1);
    ImGui::RadioButton("Remove (Ctrl)", &selection_mode_, 2);
    ImGui::RadioButton("Toggle (Alt)", &selection_mode_, 3);
    
    ImGui::Separator();
    
    // Operations
    ImGui::Text("Operations:");
    if (ImGui::Button("Clear Selection", ImVec2(120, 0))) {
      if (selection_tools_) {
        selection_tools_->ClearSelection();
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Invert Selection", ImVec2(120, 0))) {
      if (selection_tools_) {
        selection_tools_->InvertSelection();
      }
    }
    
    ImGui::SliderFloat("Grow/Shrink Dist", &grow_distance_, 0.1f, 5.0f);
    if (ImGui::Button("Grow Selection", ImVec2(120, 0))) {
      if (selection_tools_) {
        selection_tools_->GrowSelection(grow_distance_);
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Shrink Selection", ImVec2(120, 0))) {
      if (selection_tools_) {
        selection_tools_->ShrinkSelection(grow_distance_);
      }
    }
    
    ImGui::Separator();
    
    // Selection info
    if (selection_tools_) {
      const auto& selection = selection_tools_->GetCurrentSelection();
      ImGui::Text("Selected Points: %zu", selection.count);
      
      if (!selection.IsEmpty()) {
        ImGui::Text("Centroid:");
        ImGui::Text("  X: %.3f", selection.centroid.x);
        ImGui::Text("  Y: %.3f", selection.centroid.y);
        ImGui::Text("  Z: %.3f", selection.centroid.z);
        
        ImGui::Text("Bounds:");
        ImGui::Text("  Min: (%.2f, %.2f, %.2f)", 
                    selection.min_bounds.x, selection.min_bounds.y, selection.min_bounds.z);
        ImGui::Text("  Max: (%.2f, %.2f, %.2f)", 
                    selection.max_bounds.x, selection.max_bounds.y, selection.max_bounds.z);
      }
      
      int hovered = selection_tools_->GetHoveredPoint();
      if (hovered >= 0) {
        ImGui::Text("Hovered Point: %d", hovered);
      } else {
        ImGui::Text("Hovered Point: None");
      }
    }
    
    ImGui::Separator();
    
    // Highlight controls
    ImGui::Text("Highlight Options:");
    ImGui::ColorEdit3("Selection Color", &highlight_color_[0]);
    ImGui::SliderFloat("Point Size Mult", &highlight_size_, 1.0f, 4.0f);
    
    if (ImGui::Button("Apply Highlight", ImVec2(120, 0))) {
      ApplyHighlight();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear Highlights", ImVec2(120, 0))) {
      ClearHighlights();
    }
    
    ImGui::Separator();
    
    // Point cloud controls
    ImGui::Text("Display Options:");
    if (ImGui::SliderFloat("Point Size", &point_size_, 1.0f, 10.0f)) {
      if (point_cloud_) {
        point_cloud_->SetPointSize(point_size_);
      }
    }
    
    if (ImGui::SliderFloat("Opacity", &opacity_, 0.1f, 1.0f)) {
      if (point_cloud_) {
        point_cloud_->SetOpacity(opacity_);
      }
    }
    
    ImGui::Separator();
    
    // Instructions
    ImGui::TextWrapped("Instructions:");
    ImGui::TextWrapped("• Select tool above");
    ImGui::TextWrapped("• Left click/drag in 3D view");
    ImGui::TextWrapped("• Right click: rotate camera");
    ImGui::TextWrapped("• Scroll: zoom");
    ImGui::TextWrapped("• Middle click: pan");
    
    if (current_tool_ == 3) { // Lasso
      ImGui::TextWrapped("• Lasso: click and drag to draw");
    }
  }
  
  // Setters
  void SetSelectionTools(SelectionTools* tools) { selection_tools_ = tools; }
  void SetPointCloud(PointCloud* pc) { point_cloud_ = pc; }
  void SetFileInfo(const std::string& filename, size_t points) {
    filename_ = filename;
    total_points_ = points;
  }
  
  // Getters
  int GetCurrentTool() const { return current_tool_; }
  SelectionMode GetSelectionMode() const { return static_cast<SelectionMode>(selection_mode_); }
  
  void ApplyHighlight() {
    if (!selection_tools_ || !point_cloud_) return;
    
    const auto& selection = selection_tools_->GetCurrentSelection();
    if (!selection.IsEmpty()) {
      point_cloud_->HighlightPoints(
        selection.indices,
        highlight_color_,
        "manual_highlight",
        highlight_size_
      );
    }
  }
  
  void ClearHighlights() {
    if (point_cloud_) {
      point_cloud_->ClearHighlights("manual_highlight");
    }
  }
  
 private:
  SelectionTools* selection_tools_ = nullptr;
  PointCloud* point_cloud_ = nullptr;
  
  std::string filename_;
  size_t total_points_ = 0;
  
  int current_tool_ = 0;     // 0=None, 1=Point, 2=Rectangle, 3=Lasso, 4=Radius
  int selection_mode_ = 0;   // 0=Replace, 1=Add, 2=Remove, 3=Toggle
  float grow_distance_ = 0.5f;
  glm::vec3 highlight_color_ = glm::vec3(1.0f, 1.0f, 0.0f);
  float highlight_size_ = 2.0f;
  
  float point_size_ = 2.0f;
  float opacity_ = 1.0f;
};

// Enhanced scene manager with selection handling
class PCDSelectionSceneManager : public GlSceneManager {
 public:
  PCDSelectionSceneManager() : GlSceneManager("PCD Viewer with Selection") {
    SetShowRenderingInfo(true);
    SetBackgroundColor(0.05f, 0.05f, 0.08f, 1.0f);
    SetNoTitleBar(true);
  }
  
  void SetSelectionTools(SelectionTools* tools) { selection_tools_ = tools; }
  void SetControlPanel(PCDSelectionPanel* panel) { control_panel_ = panel; }
  void SetPointCloud(PointCloud* pc) { point_cloud_ = pc; }
  
  void Draw() override {
    // Handle mouse input for selection before drawing
    if (selection_tools_ && control_panel_) {
      HandleSelectionInput();
    }
    
    // Draw the scene
    GlSceneManager::Draw();
    
    // Draw selection overlays after scene
    DrawSelectionOverlay();
  }
  
 private:
  void HandleSelectionInput() {
    ImGuiIO& io = ImGui::GetIO();
    
    // Don't interfere with camera controls
    if (ImGui::IsMouseDown(1) || ImGui::IsMouseDown(2)) return; // Right or middle mouse
    
    // The 3D scene is rendered as an ImGui::Image() in the scene manager
    // We need to get the image bounds, not the general content region
    ImVec2 mouse_pos = io.MousePos;
    
    // Get the last drawn ImGui item (which should be our 3D scene image)
    ImVec2 image_min = ImGui::GetItemRectMin();
    ImVec2 image_max = ImGui::GetItemRectMax();
    ImVec2 image_size = ImVec2(image_max.x - image_min.x, image_max.y - image_min.y);
    
    // Convert to image-relative coordinates
    float local_x = mouse_pos.x - image_min.x;
    float local_y = mouse_pos.y - image_min.y;
    
    // Always update selection tools with current matrices
    auto camera = GetCamera();
    if (camera) {
      selection_tools_->SetCamera(camera);
      selection_tools_->SetViewport(0, 0, image_size.x, image_size.y);
      selection_tools_->SetProjectionMatrix(GetProjectionMatrix());
      selection_tools_->SetViewMatrix(GetViewMatrix());
      selection_tools_->SetCoordinateTransform(GetCoordinateTransform());
      
      // Check if mouse is within image bounds
      bool mouse_in_content = (local_x >= 0 && local_x < image_size.x && 
                              local_y >= 0 && local_y < image_size.y);
      
      // Always update hover (clear when outside bounds)
      if (!is_selecting_ && !ImGui::IsMouseDragging(0) && !ImGui::IsMouseDragging(1) && !ImGui::IsMouseDragging(2)) {
        if (mouse_in_content) {
          selection_tools_->UpdateHover(local_x, local_y);
        } else {
          selection_tools_->UpdateHover(-1, -1); // Clear hover when outside
        }
      }
      
      // Only handle selection when mouse is in content area
      if (!mouse_in_content) {
        return;
      }
      
      int current_tool = control_panel_->GetCurrentTool();
      
      // Handle mouse clicks for selection
      if (ImGui::IsMouseClicked(0) && current_tool != 0) {
        selection_start_ = glm::vec2(local_x, local_y);
        is_selecting_ = true;
        
        if (current_tool == 3) { // Lasso
          lasso_points_.clear();
          lasso_points_.push_back(selection_start_);
        } else if (current_tool == 1) { // Point pick
          // Immediate selection for point picking
          SelectionMode mode = control_panel_->GetSelectionMode();
          int selected = selection_tools_->PickPoint(local_x, local_y, 10.0f);
          if (selected >= 0) {
            // For point pick, we can process immediately
            std::cout << "[Point Pick] Selected point " << selected << std::endl;
          }
          is_selecting_ = false;
        }
      }
      
      // Handle drag for lasso
      if (is_selecting_ && ImGui::IsMouseDragging(0) && current_tool == 3) {
        // Add points to lasso path (limit frequency for performance)
        static int lasso_counter = 0;
        if (++lasso_counter % 2 == 0) { // Add every 2nd frame
          lasso_points_.push_back(glm::vec2(local_x, local_y));
        }
      }
      
      // Handle mouse release for area selections
      if (ImGui::IsMouseReleased(0) && is_selecting_) {
        SelectionMode mode = control_panel_->GetSelectionMode();
        
        switch (current_tool) {
          case 2: // Rectangle
            {
              auto result = selection_tools_->SelectRectangle(
                selection_start_.x, selection_start_.y,
                local_x, local_y, mode);
              std::cout << "[Rectangle] Selected " << result.count << " points" << std::endl;
            }
            break;
            
          case 3: // Lasso
            if (lasso_points_.size() >= 3) {
              lasso_points_.push_back(glm::vec2(local_x, local_y)); // Close the polygon
              auto result = selection_tools_->SelectLasso(lasso_points_, mode);
              std::cout << "[Lasso] Selected " << result.count << " points" << std::endl;
            }
            break;
            
          case 4: // Radius
            {
              float radius = glm::length(glm::vec2(local_x, local_y) - selection_start_);
              auto result = selection_tools_->SelectRadius(
                selection_start_.x, selection_start_.y,
                radius, mode);
              std::cout << "[Radius] Selected " << result.count << " points" << std::endl;
            }
            break;
        }
        
        is_selecting_ = false;
        lasso_points_.clear();
      }
    }
  }
  
  void DrawSelectionOverlay() {
    if (!is_selecting_) return;
    
    int current_tool = control_panel_->GetCurrentTool();
    if (current_tool <= 1) return; // No overlay for None or Point Pick
    
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 window_pos = ImGui::GetWindowPos();
    ImVec2 mouse_pos = ImGui::GetIO().MousePos;
    
    switch (current_tool) {
      case 2: // Rectangle
        draw_list->AddRect(
          ImVec2(window_pos.x + selection_start_.x, window_pos.y + selection_start_.y),
          mouse_pos,
          IM_COL32(255, 255, 0, 150), 0.0f, 0, 2.0f);
        break;
        
      case 3: // Lasso
        if (lasso_points_.size() > 1) {
          for (size_t i = 1; i < lasso_points_.size(); ++i) {
            draw_list->AddLine(
              ImVec2(window_pos.x + lasso_points_[i-1].x, 
                     window_pos.y + lasso_points_[i-1].y),
              ImVec2(window_pos.x + lasso_points_[i].x, 
                     window_pos.y + lasso_points_[i].y),
              IM_COL32(255, 255, 0, 200), 2.0f);
          }
          // Draw line to current mouse position
          if (!lasso_points_.empty()) {
            draw_list->AddLine(
              ImVec2(window_pos.x + lasso_points_.back().x, 
                     window_pos.y + lasso_points_.back().y),
              mouse_pos,
              IM_COL32(255, 255, 0, 100), 2.0f);
          }
        }
        break;
        
      case 4: // Radius
        {
          float radius = glm::length(glm::vec2(
            mouse_pos.x - window_pos.x, mouse_pos.y - window_pos.y) - selection_start_);
          draw_list->AddCircle(
            ImVec2(window_pos.x + selection_start_.x, window_pos.y + selection_start_.y),
            radius, IM_COL32(255, 255, 0, 150), 64, 2.0f);
        }
        break;
    }
  }
  
  SelectionTools* selection_tools_ = nullptr;
  PCDSelectionPanel* control_panel_ = nullptr;
  PointCloud* point_cloud_ = nullptr;
  bool is_selecting_ = false;
  glm::vec2 selection_start_;
  std::vector<glm::vec2> lasso_points_;
};

int main(int argc, char* argv[]) {
  // Check command line arguments
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_pcd_file>" << std::endl;
    std::cerr << "Example: " << argv[0] << " /path/to/pointcloud.pcd" << std::endl;
    return 1;
  }

  std::string pcd_file = argv[1];

  // Load PCD file - determine point type from fields
  std::cout << "\n=== Loading PCD File ===" << std::endl;
  std::cout << "File path: " << pcd_file << std::endl;
  
  // First, try to get file info using PCLPointCloud2 for detailed metadata
  pcl::PCLPointCloud2 cloud_blob;
  if (pcl::io::loadPCDFile(pcd_file, cloud_blob) != 0) {
    std::cerr << "Error: Could not load PCD file: " << pcd_file << std::endl;
    return 1;
  }
  
  std::cout << "\n=== PCD File Metadata ===" << std::endl;
  std::cout << "Fields: ";
  for (size_t i = 0; i < cloud_blob.fields.size(); ++i) {
    std::cout << cloud_blob.fields[i].name;
    if (i < cloud_blob.fields.size() - 1) std::cout << ", ";
  }
  std::cout << std::endl;
  std::cout << "Width: " << cloud_blob.width << std::endl;
  std::cout << "Height: " << cloud_blob.height << std::endl;
  std::cout << "Total points: " << cloud_blob.width * cloud_blob.height << std::endl;
  std::cout << "Is organized: " << (cloud_blob.height > 1 ? "true" : "false") << std::endl;
  
  // Check which fields exist
  bool has_rgb_field = false;
  bool has_intensity_field = false;
  
  for (const auto& field : cloud_blob.fields) {
    if (field.name == "rgb" || field.name == "rgba") {
      has_rgb_field = true;
    }
    if (field.name == "intensity" || field.name == "Intensity" || field.name == "i") {
      has_intensity_field = true;
    }
  }
  
  std::cout << "RGB: " << (has_rgb_field ? "yes" : "no") << std::endl;
  std::cout << "Intensity: " << (has_intensity_field ? "yes" : "no") << std::endl;
  
  // Variables to store point cloud data
  std::vector<glm::vec3> points_3d;
  std::vector<glm::vec3> colors_rgb;
  std::vector<glm::vec4> points_4d;
  bool has_colors = false;
  bool has_intensity = false;
  
  // Try loading with different point types based on available fields
  if (has_rgb_field) {
    // Load as PointXYZRGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file, *cloud_rgb) == 0) {
      std::cout << "\n=== Loaded Point Cloud Info ===" << std::endl;
      std::cout << "Successfully loaded " << cloud_rgb->points.size() << " points" << std::endl;
      std::cout << "Point format: XYZRGB" << std::endl;
      
      points_3d.reserve(cloud_rgb->points.size());
      colors_rgb.reserve(cloud_rgb->points.size());
      has_colors = true;
      
      // Extract RGB points with true color support
      for (const auto& pt : cloud_rgb->points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
          points_3d.push_back(glm::vec3(pt.x, pt.y, pt.z));
          colors_rgb.push_back(glm::vec3(
            static_cast<float>(pt.r) / 255.0f,
            static_cast<float>(pt.g) / 255.0f,
            static_cast<float>(pt.b) / 255.0f
          ));
        }
      }
    }
  }
  
  // If RGB loading failed or not available, try XYZI
  if (points_3d.empty() && has_intensity_field) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud_xyzi) == 0) {
      std::cout << "\n=== Loaded Point Cloud Info ===" << std::endl;
      std::cout << "Successfully loaded " << cloud_xyzi->points.size() << " points" << std::endl;
      std::cout << "Point format: XYZI" << std::endl;
      
      points_4d.reserve(cloud_xyzi->points.size());
      has_intensity = true;
      
      // Find intensity range for normalization
      float min_intensity = std::numeric_limits<float>::max();
      float max_intensity = std::numeric_limits<float>::lowest();
      
      for (const auto& pt : cloud_xyzi->points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
          min_intensity = std::min(min_intensity, pt.intensity);
          max_intensity = std::max(max_intensity, pt.intensity);
        }
      }
      
      float intensity_range = max_intensity - min_intensity;
      if (intensity_range < 0.001f) {
        intensity_range = 1.0f;
        min_intensity = 0.0f;
      }
      
      // Convert to internal format with normalized intensity in w component
      for (const auto& pt : cloud_xyzi->points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
          float normalized_intensity = (pt.intensity - min_intensity) / intensity_range;
          points_4d.push_back(glm::vec4(pt.x, pt.y, pt.z, normalized_intensity));
        }
      }
    }
  }
  
  // If both failed, try basic XYZ
  if (points_3d.empty() && points_4d.empty()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud_xyz) == 0) {
      std::cout << "\n=== Loaded Point Cloud Info ===" << std::endl;
      std::cout << "Successfully loaded " << cloud_xyz->points.size() << " points" << std::endl;
      std::cout << "Point format: XYZ" << std::endl;
      
      points_4d.reserve(cloud_xyz->points.size());
      
      // Convert to internal format
      for (const auto& pt : cloud_xyz->points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
          points_4d.push_back(glm::vec4(pt.x, pt.y, pt.z, 0.0f));
        }
      }
    } else {
      std::cerr << "Error: Could not load PCD file with any supported format" << std::endl;
      return 1;
    }
  }
  
  if (points_3d.empty() && points_4d.empty()) {
    std::cerr << "Error: No valid points found in PCD file" << std::endl;
    return 1;
  }

  size_t total_points = has_colors ? points_3d.size() : points_4d.size();
  std::cout << "\nPoints to render: " << total_points << std::endl;
  
  // Create viewer
  Viewer viewer("PCD Viewer with Selection", 1400, 900);
  viewer.SetBackgroundColor(0.1f, 0.1f, 0.15f, 1.0f);
  viewer.EnableKeyboardNav(true);
  viewer.EnableDocking(true);

  // Create box layout
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // Create enhanced scene manager with selection
  auto gl_sm = std::make_shared<PCDSelectionSceneManager>();
  gl_sm->SetAutoLayout(true);
  gl_sm->SetFlexGrow(1.0f);
  gl_sm->SetFlexShrink(0.0f);
  
  // Create point cloud with appropriate color mode
  auto point_cloud = std::make_unique<PointCloud>();
  point_cloud->SetPointSize(2.0f);
  point_cloud->SetOpacity(1.0f);
  point_cloud->SetRenderMode(PointMode::kPoint);
  
  if (has_colors) {
    // Use true RGB colors
    point_cloud->SetPoints(points_3d, colors_rgb);
    std::cout << "Using true RGB color visualization" << std::endl;
  } else if (has_intensity) {
    // Use intensity/scalar field
    point_cloud->SetScalarRange(0.0f, 1.0f);
    point_cloud->SetPoints(points_4d, PointCloud::ColorMode::kScalarField);
    std::cout << "Using intensity field visualization" << std::endl;
  } else {
    // Use height field
    point_cloud->SetPoints(points_4d, PointCloud::ColorMode::kHeightField);
    std::cout << "Using height field visualization" << std::endl;
  }

  // Get raw pointer before moving for selection tools
  PointCloud* pc_ptr = point_cloud.get();
  gl_sm->AddOpenGLObject("point_cloud", std::move(point_cloud));
  
  // Add a grid for reference
  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // Create selection tools
  auto selection_tools = std::make_unique<SelectionTools>();
  selection_tools->SetPointCloud(pc_ptr);
  
  // Set up callbacks
  selection_tools->SetSelectionCallback([](const SelectionResult& result) {
    std::cout << "[Selection Changed] " << result.count << " points selected";
    if (!result.IsEmpty()) {
      std::cout << " | Centroid: (" 
                << std::fixed << std::setprecision(2)
                << result.centroid.x << ", " 
                << result.centroid.y << ", " 
                << result.centroid.z << ")";
    }
    std::cout << std::endl;
  });
  
  selection_tools->SetHoverCallback([pc_ptr](int point_index) {
    if (point_index >= 0) {
      pc_ptr->HighlightPoint(
        static_cast<size_t>(point_index),
        glm::vec3(1.0f, 1.0f, 0.0f), // Yellow for hover
        "hover",
        1.5f
      );
    } else {
      pc_ptr->ClearHighlights("hover");
    }
  });

  // Create selection control panel
  auto selection_panel = std::make_shared<PCDSelectionPanel>();
  selection_panel->SetSelectionTools(selection_tools.get());
  selection_panel->SetPointCloud(pc_ptr);
  selection_panel->SetFileInfo(pcd_file, total_points);
  selection_panel->SetWidth(280);
  selection_panel->SetFlexShrink(0.0f);

  // Connect components
  gl_sm->SetSelectionTools(selection_tools.get());
  gl_sm->SetControlPanel(selection_panel.get());
  gl_sm->SetPointCloud(pc_ptr);

  // Add to layout
  box->AddChild(selection_panel);
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  std::cout << "\n=== Starting Interactive Viewer ===" << std::endl;
  std::cout << "Use the left panel to select tools and interact with the point cloud" << std::endl;
  std::cout << "Camera controls: Right-click to rotate, scroll to zoom, middle-click to pan" << std::endl;

  viewer.Show();

  return 0;
}