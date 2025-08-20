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
#include <filesystem>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "imview/box.hpp"
#include "imview/viewer.hpp"

#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/selection/selection_tools.hpp"
#include "gldraw/pcl_bridge/pcl_loader.hpp"

using namespace quickviz;

// Selection control panel for PCD viewer
class PCDSelectionPanel : public Panel {
 public:
  PCDSelectionPanel(const pcl_bridge::PointCloudMetadata& metadata) 
      : Panel("Selection Tools"), metadata_(metadata) {
    SetAutoLayout(true);
  }
  
  void Draw() override {
    ImGui::Text("PCD Selection Tools");
    ImGui::Separator();
    
    // File info
    ImGui::Text("File: %s", std::filesystem::path(metadata_.filename).filename().c_str());
    ImGui::Text("Points: %zu", metadata_.point_count);
    ImGui::Text("Format: %s", metadata_.format.c_str());
    ImGui::Text("PCL Type: %s", metadata_.detected_pcl_type.c_str());
    ImGui::Separator();
    
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
  
  pcl_bridge::PointCloudMetadata metadata_;
  
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
    std::cerr << "Usage: " << argv[0] << " <path_to_point_cloud_file>" << std::endl;
    std::cerr << "Supported formats: .pcd, .ply" << std::endl;
    return 1;
  }

  std::string point_cloud_file = argv[1];

  std::cout << "\n=== QuickViz PCD Selection Tool ===" << std::endl;
  std::cout << "File: " << point_cloud_file << std::endl;

  try {
    // First, analyze the file to understand its structure
    std::cout << "\n=== Analyzing Point Cloud File ===" << std::endl;
    auto analysis_metadata = pcl_bridge::PointCloudLoader::AnalyzeFields(point_cloud_file);
    
    std::cout << "Format: " << analysis_metadata.format << std::endl;
    std::cout << "File size: " << analysis_metadata.file_size_mb << " MB" << std::endl;
    std::cout << "Recommended PCL type: " << analysis_metadata.GetRecommendedPCLType() << std::endl;
    
    std::cout << "\nDetected fields:" << std::endl;
    std::cout << "  XYZ: " << (analysis_metadata.fields.HasXYZ() ? "yes" : "no") << std::endl;
    std::cout << "  RGB: " << (analysis_metadata.fields.HasRGBColor() ? "yes" : "no") << std::endl;
    std::cout << "  RGBA: " << (analysis_metadata.fields.HasRGBAColor() ? "yes" : "no") << std::endl;
    std::cout << "  Intensity: " << (analysis_metadata.fields.has_intensity ? "yes" : "no") << std::endl;
    std::cout << "  Normals: " << (analysis_metadata.fields.HasNormals() ? "yes" : "no") << std::endl;
    // Load point cloud using the PCL loader
    std::cout << "\n=== Loading Point Cloud ===" << std::endl;
    
    // Load based on detected type
    pcl_bridge::PointCloudMetadata metadata;
    std::string optimal_type = analysis_metadata.GetRecommendedPCLType();
    
    // Variables to store the converted point data
    std::vector<glm::vec3> points_3d;
    std::vector<glm::vec3> colors_rgb;
    std::vector<glm::vec4> points_4d;
    bool use_rgb_colors = false;
    bool use_intensity = false;
    
    std::cout << "Loading as " << optimal_type << "..." << std::endl;
    
    if (optimal_type == "PointXYZRGB") {
      // Load with RGB colors
      auto [pcl_cloud, load_meta] = pcl_bridge::PointCloudLoader::LoadToPCL<pcl::PointXYZRGB>(
          point_cloud_file, pcl_bridge::PointCloudLoader::Format::kAutoDetect);
      metadata = load_meta;
      
      std::cout << "Converting " << pcl_cloud->points.size() << " RGB points to renderer format..." << std::endl;
      
      // Convert to renderer format
      points_3d.reserve(pcl_cloud->points.size());
      colors_rgb.reserve(pcl_cloud->points.size());
      
      for (const auto& pt : pcl_cloud->points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
          points_3d.push_back(glm::vec3(pt.x, pt.y, pt.z));
          colors_rgb.push_back(glm::vec3(pt.r / 255.0f, pt.g / 255.0f, pt.b / 255.0f));
        }
      }
      use_rgb_colors = true;
      std::cout << "Converted " << points_3d.size() << " RGB points" << std::endl;
      
    } else if (optimal_type == "PointXYZRGBA") {
      // Load with RGBA colors (treat as RGB)
      auto [pcl_cloud, load_meta] = pcl_bridge::PointCloudLoader::LoadToPCL<pcl::PointXYZRGBA>(
          point_cloud_file, pcl_bridge::PointCloudLoader::Format::kAutoDetect);
      metadata = load_meta;
      
      std::cout << "Converting " << pcl_cloud->points.size() << " RGBA points to renderer format..." << std::endl;
      
      points_3d.reserve(pcl_cloud->points.size());
      colors_rgb.reserve(pcl_cloud->points.size());
      
      for (const auto& pt : pcl_cloud->points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
          points_3d.push_back(glm::vec3(pt.x, pt.y, pt.z));
          colors_rgb.push_back(glm::vec3(pt.r / 255.0f, pt.g / 255.0f, pt.b / 255.0f));
        }
      }
      use_rgb_colors = true;
      std::cout << "Converted " << points_3d.size() << " RGBA points" << std::endl;
      
    } else if (optimal_type == "PointXYZI") {
      // Load with intensity
      auto [pcl_cloud, load_meta] = pcl_bridge::PointCloudLoader::LoadToPCL<pcl::PointXYZI>(
          point_cloud_file, pcl_bridge::PointCloudLoader::Format::kAutoDetect);
      metadata = load_meta;
      
      std::cout << "Converting " << pcl_cloud->points.size() << " intensity points to renderer format..." << std::endl;
      
      // Convert to renderer format with intensity normalization
      points_4d.reserve(pcl_cloud->points.size());
      
      float min_intensity = std::numeric_limits<float>::max();
      float max_intensity = std::numeric_limits<float>::lowest();
      
      // Find intensity range
      for (const auto& pt : pcl_cloud->points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) && !std::isnan(pt.intensity)) {
          min_intensity = std::min(min_intensity, pt.intensity);
          max_intensity = std::max(max_intensity, pt.intensity);
        }
      }
      
      float intensity_range = max_intensity - min_intensity;
      if (intensity_range < 0.001f) {
        intensity_range = 1.0f;
        min_intensity = 0.0f;
      }
      
      std::cout << "Intensity range: [" << min_intensity << ", " << max_intensity << "]" << std::endl;
      
      for (const auto& pt : pcl_cloud->points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
          float normalized_intensity = std::isnan(pt.intensity) ? 0.0f : 
                                     (pt.intensity - min_intensity) / intensity_range;
          points_4d.push_back(glm::vec4(pt.x, pt.y, pt.z, normalized_intensity));
        }
      }
      use_intensity = true;
      std::cout << "Converted " << points_4d.size() << " intensity points" << std::endl;
      
    } else {
      // Load as XYZ (default)
      auto [pcl_cloud, load_meta] = pcl_bridge::PointCloudLoader::LoadToPCL<pcl::PointXYZ>(
          point_cloud_file, pcl_bridge::PointCloudLoader::Format::kAutoDetect);
      metadata = load_meta;
      
      std::cout << "Converting " << pcl_cloud->points.size() << " XYZ points to renderer format..." << std::endl;
      
      // Convert to renderer format (use Z for height-based coloring)
      points_4d.reserve(pcl_cloud->points.size());
      
      for (const auto& pt : pcl_cloud->points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
          points_4d.push_back(glm::vec4(pt.x, pt.y, pt.z, pt.z));  // Use Z as scalar for height coloring
        }
      }
      
      std::cout << "Converted " << points_4d.size() << " XYZ points" << std::endl;
    }
    
    std::cout << "\n=== Load Results ===" << std::endl;
    std::cout << "Successfully loaded " << metadata.point_count << " points" << std::endl;
    std::cout << "Detected PCL type: " << metadata.detected_pcl_type << std::endl;
    std::cout << "Bounding box: [" << metadata.min_bounds.x << ", " << metadata.min_bounds.y << ", " << metadata.min_bounds.z 
              << "] to [" << metadata.max_bounds.x << ", " << metadata.max_bounds.y << ", " << metadata.max_bounds.z << "]" << std::endl;

    // Calculate some statistics
    glm::vec3 size = metadata.max_bounds - metadata.min_bounds;
    glm::vec3 center = (metadata.min_bounds + metadata.max_bounds) * 0.5f;
    
    std::cout << "Point cloud size: " << size.x << " x " << size.y << " x " << size.z << std::endl;
    std::cout << "Point cloud center: (" << center.x << ", " << center.y << ", " << center.z << ")" << std::endl;
    
    // Create viewer for visualization (this initializes OpenGL context)
    std::cout << "\n=== Creating Visualization ===" << std::endl;
    Viewer viewer("PCD Viewer with Selection", 1400, 900);
    viewer.SetBackgroundColor(0.1f, 0.1f, 0.15f, 1.0f);
    viewer.EnableKeyboardNav(true);
    viewer.EnableDocking(true);

    // Create main container box
    auto main_box = std::make_shared<Box>("main_container");
    main_box->SetFlexDirection(Styling::FlexDirection::kRow);
    main_box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
    main_box->SetAlignItems(Styling::AlignItems::kStretch);

    // Create enhanced scene manager with selection
    auto gl_sm = std::make_shared<PCDSelectionSceneManager>();
    gl_sm->SetAutoLayout(true);
    gl_sm->SetFlexGrow(0.85f);      // Allow growth for 3D view
    gl_sm->SetFlexShrink(1.0f);     // Allow shrinking if needed
    
    // NOW create the PointCloud object (OpenGL context exists)
    std::cout << "Creating PointCloud object with OpenGL context available..." << std::endl;
    auto point_cloud = std::make_unique<PointCloud>();
    point_cloud->SetPointSize(2.0f);
    point_cloud->SetOpacity(1.0f);
    point_cloud->SetRenderMode(PointMode::kPoint);
    
    // Set points based on what we loaded
    if (use_rgb_colors) {
      std::cout << "Setting " << points_3d.size() << " points with RGB colors..." << std::endl;
      point_cloud->SetPoints(points_3d, colors_rgb);
      std::cout << "Using RGB coloring" << std::endl;
    } else if (use_intensity) {
      std::cout << "Setting " << points_4d.size() << " points with intensity coloring..." << std::endl;
      point_cloud->SetScalarRange(0.0f, 1.0f);  // Intensity is normalized
      point_cloud->SetPoints(points_4d, PointCloud::ColorMode::kScalarField);
      std::cout << "Using intensity-based coloring" << std::endl;
    } else {
      std::cout << "Setting " << points_4d.size() << " points with height-based coloring..." << std::endl;
      point_cloud->SetScalarRange(metadata.min_bounds.z, metadata.max_bounds.z);  // Height-based
      point_cloud->SetPoints(points_4d, PointCloud::ColorMode::kHeightField);
      std::cout << "Using height-based coloring" << std::endl;
    }

    // Get raw pointer before moving for selection tools
    PointCloud* pc_ptr = point_cloud.get();
    gl_sm->AddOpenGLObject("loaded_point_cloud", std::move(point_cloud));
    
    // Add a reference grid
    glm::vec3 bounds_size = metadata.max_bounds - metadata.min_bounds;
    auto grid = std::make_unique<Grid>(
        std::max(bounds_size.x, bounds_size.y) * 0.1f,  // Grid spacing based on point cloud size
        std::max(bounds_size.x, bounds_size.y),         // Grid size
        glm::vec3(0.7f, 0.7f, 0.7f)      // Grid color
    );
    gl_sm->AddOpenGLObject("reference_grid", std::move(grid));

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
    auto selection_panel = std::make_shared<PCDSelectionPanel>(metadata);
    selection_panel->SetSelectionTools(selection_tools.get());
    selection_panel->SetPointCloud(pc_ptr);
    selection_panel->SetAutoLayout(true);
    selection_panel->SetFlexGrow(0.15f);     // Panel takes less space
    selection_panel->SetFlexShrink(0.0f);   // Don't shrink below basis

    // Connect components
    gl_sm->SetSelectionTools(selection_tools.get());
    gl_sm->SetControlPanel(selection_panel.get());
    gl_sm->SetPointCloud(pc_ptr);

    // Add components to main container
    main_box->AddChild(selection_panel);
    main_box->AddChild(gl_sm);
    
    // Add to viewer
    viewer.AddSceneObject(main_box);

    std::cout << "\n=== Starting Interactive Viewer ===" << std::endl;
    std::cout << "Use the left panel to select tools and interact with the point cloud" << std::endl;
    std::cout << "Camera controls: Right-click to rotate, scroll to zoom, middle-click to pan" << std::endl;

    viewer.Show();

  } catch (const pcl_bridge::FileNotFoundException& e) {
    std::cerr << "Error: File not found - " << e.what() << std::endl;
    return 1;
  } catch (const pcl_bridge::UnsupportedFormatException& e) {
    std::cerr << "Error: Unsupported format - " << e.what() << std::endl;
    std::cerr << "Supported formats: ";
    auto extensions = pcl_bridge::PointCloudLoader::GetSupportedExtensions();
    for (size_t i = 0; i < extensions.size(); ++i) {
      std::cerr << extensions[i];
      if (i < extensions.size() - 1) std::cerr << ", ";
    }
    std::cerr << std::endl;
    return 1;
  } catch (const pcl_bridge::CorruptedFileException& e) {
    std::cerr << "Error: Corrupted file - " << e.what() << std::endl;
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}