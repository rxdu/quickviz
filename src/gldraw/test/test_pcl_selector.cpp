/*
 * test_pcl_loader_render.cpp
 *
 * Created on: Dec 2024
 * Description: Test PCL loader by loading and rendering various point cloud
 * files
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <vector>
#include <filesystem>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "imview/box.hpp"
#include "imview/viewer.hpp"
#include "imview/panel.hpp"

#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/pcl_bridge/pcl_loader.hpp"

using namespace quickviz;

// Simple panel to display point cloud information
class PointCloudInfoPanel : public Panel {
 public:
  PointCloudInfoPanel(const std::string& name,
                      const pcl_bridge::PointCloudMetadata& metadata)
      : Panel(name), metadata_(metadata) {}

  void Draw() override {
    // Use explicit window begin/end to control the title
    ImGui::Begin("Point Cloud Info");

    // ImGui::Text("Point Cloud Information");
    // ImGui::Separator();
    ImGui::Dummy({0.0f, 5.0f});  // Add some space before the title

    ImGui::Text("File: %s",
                std::filesystem::path(metadata_.filename).filename().c_str());
    ImGui::Text("Format: %s", metadata_.format.c_str());
    ImGui::Text("PCL Type: %s", metadata_.detected_pcl_type.c_str());
    ImGui::Text("Points: %zu", metadata_.point_count);
    ImGui::Text("File Size: %.2f MB", metadata_.file_size_mb);

    ImGui::Separator();
    ImGui::Text("Available Fields:");
    ImGui::BulletText("XYZ: %s", metadata_.fields.HasXYZ() ? "Yes" : "No");
    ImGui::BulletText("RGB: %s", metadata_.fields.HasRGBColor() ? "Yes" : "No");
    ImGui::BulletText("RGBA: %s",
                      metadata_.fields.HasRGBAColor() ? "Yes" : "No");
    ImGui::BulletText("Intensity: %s",
                      metadata_.fields.has_intensity ? "Yes" : "No");
    ImGui::BulletText("Normals: %s",
                      metadata_.fields.HasNormals() ? "Yes" : "No");

    ImGui::Separator();
    ImGui::Text("Bounding Box:");
    ImGui::Text("  Min: (%.2f, %.2f, %.2f)", metadata_.min_bounds.x,
                metadata_.min_bounds.y, metadata_.min_bounds.z);
    ImGui::Text("  Max: (%.2f, %.2f, %.2f)", metadata_.max_bounds.x,
                metadata_.max_bounds.y, metadata_.max_bounds.z);

    glm::vec3 panel_size = metadata_.max_bounds - metadata_.min_bounds;
    ImGui::Text("  Size: (%.2f, %.2f, %.2f)", panel_size.x, panel_size.y,
                panel_size.z);

    glm::vec3 center = (metadata_.min_bounds + metadata_.max_bounds) * 0.5f;
    ImGui::Text("  Center: (%.2f, %.2f, %.2f)", center.x, center.y, center.z);

    ImGui::End();
  }

 private:
  pcl_bridge::PointCloudMetadata metadata_;
};

// Forward declaration
class PointCloudToolPanel;

// Custom scene manager that can update the tool panel
class InteractiveSceneManager : public GlSceneManager {
 public:
  InteractiveSceneManager(const std::string& name, Mode mode = Mode::k3D)
      : GlSceneManager(name, mode) {}
  
  void SetToolPanel(PointCloudToolPanel* panel) {
    tool_panel_ = panel;
  }
  
  void Draw() override;
  
 private:
  PointCloudToolPanel* tool_panel_ = nullptr;
};

class PointCloudToolPanel : public Panel {
 public:
  PointCloudToolPanel(const std::string& name, GlSceneManager* scene_manager) 
      : Panel(name), scene_manager_(scene_manager) {}

  void Draw() override {
    // Use explicit window begin/end to control the title
    ImGui::Begin("Point Cloud Tools");

    ImGui::Text("Mouse Tracking");
    ImGui::Separator();
    
    if (mouse_info_.valid) {
      ImGui::Text("Screen Position: (%.1f, %.1f)", 
                 mouse_info_.screen_pos.x, mouse_info_.screen_pos.y);
      
      ImGui::Separator();
      ImGui::Text("Ray Origin:");
      ImGui::Text("  X: %.3f", mouse_info_.ray.origin.x);
      ImGui::Text("  Y: %.3f", mouse_info_.ray.origin.y);
      ImGui::Text("  Z: %.3f", mouse_info_.ray.origin.z);
      
      ImGui::Text("Ray Direction:");
      ImGui::Text("  X: %.3f", mouse_info_.ray.direction.x);
      ImGui::Text("  Y: %.3f", mouse_info_.ray.direction.y);
      ImGui::Text("  Z: %.3f", mouse_info_.ray.direction.z);
      
      // Calculate a point along the ray (e.g., at distance 10 units)
      float distance = 10.0f;
      glm::vec3 point_on_ray = mouse_info_.ray.origin + mouse_info_.ray.direction * distance;
      ImGui::Separator();
      ImGui::Text("Point at distance %.1f:", distance);
      ImGui::Text("  (%.2f, %.2f, %.2f)", 
                 point_on_ray.x, point_on_ray.y, point_on_ray.z);
    } else {
      ImGui::Text("Mouse not in scene");
    }
    
    ImGui::Separator();
    ImGui::Text("Selection Tools");
    ImGui::Text("Coming soon...");

    ImGui::End();
  }
  
  struct MouseInfo {
    bool valid = false;
    glm::vec2 screen_pos;
    GlSceneManager::MouseRay ray;
  };
  
  void UpdateMouseInfo(const MouseInfo& info) {
    mouse_info_ = info;
  }
  
 private:
  GlSceneManager* scene_manager_ = nullptr;
  MouseInfo mouse_info_;
};

// Implementation of InteractiveSceneManager::Draw
void InteractiveSceneManager::Draw() {
  Begin();

  // Store the mouse tracking state before calling RenderInsideWindow
  PointCloudToolPanel::MouseInfo mouse_info;
  
  // Get current mouse state directly using ImGui
  ImGuiIO& io = ImGui::GetIO();
  ImVec2 content_size = ImGui::GetContentRegionAvail();
  
  // Calculate mouse position relative to this window's content area
  ImVec2 window_pos = ImGui::GetWindowPos();
  ImVec2 window_content_min = ImGui::GetWindowContentRegionMin();
  float local_x = io.MousePos.x - window_pos.x - window_content_min.x;
  float local_y = io.MousePos.y - window_pos.y - window_content_min.y;
  
  // Check if mouse is inside the content area and window is hovered
  bool mouse_in_content = (local_x >= 0 && local_x <= content_size.x &&
                          local_y >= 0 && local_y <= content_size.y);
  bool window_hovered = ImGui::IsWindowHovered();
  
  if (window_hovered && mouse_in_content) {
    mouse_info.valid = true;
    mouse_info.screen_pos = glm::vec2(local_x, local_y);
    mouse_info.ray = GetMouseRayInWorldSpace(local_x, local_y, 
                                            content_size.x, content_size.y);
    
    // Draw crosshair overlay
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    
    // Calculate absolute position for crosshair
    float abs_x = window_pos.x + window_content_min.x + local_x;
    float abs_y = window_pos.y + window_content_min.y + local_y;
    
    // Crosshair parameters
    const float crosshair_size = 15.0f;
    const float gap = 3.0f;
    const ImU32 color = IM_COL32(0, 255, 0, 200); // Green with some transparency
    const float thickness = 2.0f;
    
    // Draw horizontal line (with gap in middle)
    draw_list->AddLine(
        ImVec2(abs_x - crosshair_size, abs_y),
        ImVec2(abs_x - gap, abs_y),
        color, thickness);
    draw_list->AddLine(
        ImVec2(abs_x + gap, abs_y),
        ImVec2(abs_x + crosshair_size, abs_y),
        color, thickness);
    
    // Draw vertical line (with gap in middle)
    draw_list->AddLine(
        ImVec2(abs_x, abs_y - crosshair_size),
        ImVec2(abs_x, abs_y - gap),
        color, thickness);
    draw_list->AddLine(
        ImVec2(abs_x, abs_y + gap),
        ImVec2(abs_x, abs_y + crosshair_size),
        color, thickness);
    
    // Optional: Draw center dot
    draw_list->AddCircleFilled(ImVec2(abs_x, abs_y), 2.0f, IM_COL32(255, 255, 0, 255));
  }
  
  RenderInsideWindow();
  
  // Update tool panel with mouse information
  if (tool_panel_) {
    tool_panel_->UpdateMouseInfo(mouse_info);
  }
  
  End();
}

int main(int argc, char* argv[]) {
  // Check command line arguments
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_point_cloud_file>"
              << std::endl;
    std::cerr << "Supported formats: .pcd, .ply" << std::endl;
    return 1;
  }

  std::string point_cloud_file = argv[1];

  std::cout << "\n=== QuickViz PCL Loader Test ===" << std::endl;
  std::cout << "File: " << point_cloud_file << std::endl;

  try {
    // First, analyze the file to understand its structure
    std::cout << "\n=== Analyzing Point Cloud File ===" << std::endl;
    auto analysis_metadata =
        pcl_bridge::PointCloudLoader::AnalyzeFields(point_cloud_file);

    std::cout << "Format: " << analysis_metadata.format << std::endl;
    std::cout << "File size: " << analysis_metadata.file_size_mb << " MB"
              << std::endl;
    std::cout << "Recommended PCL type: "
              << analysis_metadata.GetRecommendedPCLType() << std::endl;

    std::cout << "\nDetected fields:" << std::endl;
    std::cout << "  XYZ: " << (analysis_metadata.fields.HasXYZ() ? "yes" : "no")
              << std::endl;
    std::cout << "  RGB: "
              << (analysis_metadata.fields.HasRGBColor() ? "yes" : "no")
              << std::endl;
    std::cout << "  RGBA: "
              << (analysis_metadata.fields.HasRGBAColor() ? "yes" : "no")
              << std::endl;
    std::cout << "  Intensity: "
              << (analysis_metadata.fields.has_intensity ? "yes" : "no")
              << std::endl;
    std::cout << "  Normals: "
              << (analysis_metadata.fields.HasNormals() ? "yes" : "no")
              << std::endl;

    // Load point cloud using the PCL loader with progress callback
    std::cout << "\n=== Loading Point Cloud ===" << std::endl;

    // Try without callback first to isolate the issue
    // auto progress_callback = [](float progress, const std::string& message) {
    //   std::cout << "Progress: " << static_cast<int>(progress * 100) << "% - "
    //   << message << std::endl;
    // };

    // Load based on detected type (but defer point cloud object creation until
    // after OpenGL context)
    pcl_bridge::PointCloudMetadata metadata;
    std::string optimal_type = analysis_metadata.GetRecommendedPCLType();

    // Variables to store the converted point data
    std::vector<glm::vec3> points_3d;
    std::vector<glm::vec3> colors_rgb;
    std::vector<glm::vec4> points_4d;
    bool use_rgb_colors = false;
    bool use_intensity = false;

    std::cout << "Loading as " << optimal_type << "..." << std::endl;

    try {
      std::cout << "Loading point cloud data..." << std::endl;

      if (optimal_type == "PointXYZRGB") {
        // Load with RGB colors
        auto [pcl_cloud, load_meta] =
            pcl_bridge::PointCloudLoader::LoadToPCL<pcl::PointXYZRGB>(
                point_cloud_file,
                pcl_bridge::PointCloudLoader::Format::kAutoDetect);
        metadata = load_meta;

        std::cout << "Converting " << pcl_cloud->points.size()
                  << " RGB points to renderer format..." << std::endl;

        // Convert to renderer format
        points_3d.reserve(pcl_cloud->points.size());
        colors_rgb.reserve(pcl_cloud->points.size());

        for (const auto& pt : pcl_cloud->points) {
          if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
            points_3d.push_back(glm::vec3(pt.x, pt.y, pt.z));
            colors_rgb.push_back(
                glm::vec3(pt.r / 255.0f, pt.g / 255.0f, pt.b / 255.0f));
          }
        }
        use_rgb_colors = true;
        std::cout << "Converted " << points_3d.size() << " RGB points"
                  << std::endl;

      } else if (optimal_type == "PointXYZRGBA") {
        // Load with RGBA colors (treat as RGB)
        auto [pcl_cloud, load_meta] =
            pcl_bridge::PointCloudLoader::LoadToPCL<pcl::PointXYZRGBA>(
                point_cloud_file,
                pcl_bridge::PointCloudLoader::Format::kAutoDetect);
        metadata = load_meta;

        std::cout << "Converting " << pcl_cloud->points.size()
                  << " RGBA points to renderer format..." << std::endl;

        points_3d.reserve(pcl_cloud->points.size());
        colors_rgb.reserve(pcl_cloud->points.size());

        for (const auto& pt : pcl_cloud->points) {
          if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
            points_3d.push_back(glm::vec3(pt.x, pt.y, pt.z));
            colors_rgb.push_back(
                glm::vec3(pt.r / 255.0f, pt.g / 255.0f, pt.b / 255.0f));
          }
        }
        use_rgb_colors = true;
        std::cout << "Converted " << points_3d.size() << " RGBA points"
                  << std::endl;

      } else if (optimal_type == "PointXYZI") {
        // Load with intensity
        auto [pcl_cloud, load_meta] =
            pcl_bridge::PointCloudLoader::LoadToPCL<pcl::PointXYZI>(
                point_cloud_file,
                pcl_bridge::PointCloudLoader::Format::kAutoDetect);
        metadata = load_meta;

        std::cout << "Converting " << pcl_cloud->points.size()
                  << " intensity points to renderer format..." << std::endl;

        // Convert to renderer format with intensity normalization
        points_4d.reserve(pcl_cloud->points.size());

        float min_intensity = std::numeric_limits<float>::max();
        float max_intensity = std::numeric_limits<float>::lowest();

        // Find intensity range
        for (const auto& pt : pcl_cloud->points) {
          if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) &&
              !std::isnan(pt.intensity)) {
            min_intensity = std::min(min_intensity, pt.intensity);
            max_intensity = std::max(max_intensity, pt.intensity);
          }
        }

        float intensity_range = max_intensity - min_intensity;
        if (intensity_range < 0.001f) {
          intensity_range = 1.0f;
          min_intensity = 0.0f;
        }

        std::cout << "Intensity range: [" << min_intensity << ", "
                  << max_intensity << "]" << std::endl;

        for (const auto& pt : pcl_cloud->points) {
          if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
            float normalized_intensity =
                std::isnan(pt.intensity)
                    ? 0.0f
                    : (pt.intensity - min_intensity) / intensity_range;
            points_4d.push_back(
                glm::vec4(pt.x, pt.y, pt.z, normalized_intensity));
          }
        }
        use_intensity = true;
        std::cout << "Converted " << points_4d.size() << " intensity points"
                  << std::endl;

      } else {
        // Load as XYZ (default)
        auto [pcl_cloud, load_meta] =
            pcl_bridge::PointCloudLoader::LoadToPCL<pcl::PointXYZ>(
                point_cloud_file,
                pcl_bridge::PointCloudLoader::Format::kAutoDetect);
        metadata = load_meta;

        std::cout << "Converting " << pcl_cloud->points.size()
                  << " XYZ points to renderer format..." << std::endl;

        // Convert to renderer format (use Z for height-based coloring)
        points_4d.reserve(pcl_cloud->points.size());

        for (const auto& pt : pcl_cloud->points) {
          if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
            points_4d.push_back(
                glm::vec4(pt.x, pt.y, pt.z,
                          pt.z));  // Use Z as scalar for height coloring
          }
        }

        std::cout << "Converted " << points_4d.size() << " XYZ points"
                  << std::endl;
      }

    } catch (const std::exception& e) {
      std::cerr << "Exception during loading: " << e.what() << std::endl;
      throw;
    }

    std::cout << "Loading process completed, proceeding to display results..."
              << std::endl;

    std::cout << "\n=== Load Results ===" << std::endl;
    std::cout << "Successfully loaded " << metadata.point_count << " points"
              << std::endl;
    std::cout << "Detected PCL type: " << metadata.detected_pcl_type
              << std::endl;
    std::cout << "Bounding box: [" << metadata.min_bounds.x << ", "
              << metadata.min_bounds.y << ", " << metadata.min_bounds.z
              << "] to [" << metadata.max_bounds.x << ", "
              << metadata.max_bounds.y << ", " << metadata.max_bounds.z << "]"
              << std::endl;

    // Calculate some statistics
    glm::vec3 size = metadata.max_bounds - metadata.min_bounds;
    glm::vec3 center = (metadata.min_bounds + metadata.max_bounds) * 0.5f;

    std::cout << "Point cloud size: " << size.x << " x " << size.y << " x "
              << size.z << std::endl;
    std::cout << "Point cloud center: (" << center.x << ", " << center.y << ", "
              << center.z << ")" << std::endl;

    // Create viewer for visualization (this initializes OpenGL context)
    std::cout << "\n=== Creating Visualization ===" << std::endl;
    Viewer viewer;

    // Create main container box
    auto main_box = std::make_shared<Box>("main_container");
    main_box->SetFlexDirection(Styling::FlexDirection::kRow);
    main_box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
    main_box->SetAlignItems(Styling::AlignItems::kStretch);

    // Create OpenGL scene manager for 3D visualization
    auto gl_sm = std::make_shared<InteractiveSceneManager>("Point Cloud Viewer");
    gl_sm->SetAutoLayout(true);
    gl_sm->SetNoTitleBar(false);  // Show title bar to match the panel
    // gl_sm->SetFlexBasis(600.0f);   // Base width for 3D view
    gl_sm->SetFlexGrow(0.85f);   // Allow some growth but less than panel
    gl_sm->SetFlexShrink(1.0f);  // Allow shrinking if needed

    // NOW create the PointCloud object (OpenGL context exists)
    std::cout << "Creating PointCloud object with OpenGL context available..."
              << std::endl;
    auto point_cloud = std::make_unique<PointCloud>();
    point_cloud->SetPointSize(2.0f);
    point_cloud->SetOpacity(1.0f);
    point_cloud->SetRenderMode(PointMode::kPoint);

    // Set points based on what we loaded
    if (use_rgb_colors) {
      std::cout << "Setting " << points_3d.size()
                << " points with RGB colors..." << std::endl;
      point_cloud->SetPoints(points_3d, colors_rgb);
      std::cout << "Using RGB coloring" << std::endl;
    } else if (use_intensity) {
      std::cout << "Setting " << points_4d.size()
                << " points with intensity coloring..." << std::endl;
      point_cloud->SetScalarRange(0.0f, 1.0f);  // Intensity is normalized
      point_cloud->SetPoints(points_4d, PointCloud::ColorMode::kScalarField);
      std::cout << "Using intensity-based coloring" << std::endl;
    } else {
      std::cout << "Setting " << points_4d.size()
                << " points with height-based coloring..." << std::endl;
      point_cloud->SetScalarRange(metadata.min_bounds.z,
                                  metadata.max_bounds.z);  // Height-based
      point_cloud->SetPoints(points_4d, PointCloud::ColorMode::kHeightField);
      std::cout << "Using height-based coloring" << std::endl;
    }

    // Add the point cloud to the scene
    gl_sm->AddOpenGLObject("loaded_point_cloud", std::move(point_cloud));

    // Add a reference grid
    glm::vec3 bounds_size = metadata.max_bounds - metadata.min_bounds;
    auto grid = std::make_unique<Grid>(
        std::max(bounds_size.x, bounds_size.y) *
            0.1f,  // Grid spacing based on point cloud size
        std::max(bounds_size.x, bounds_size.y),  // Grid size
        glm::vec3(0.7f, 0.7f, 0.7f)              // Grid color
    );
    gl_sm->AddOpenGLObject("reference_grid", std::move(grid));

    auto vbox = std::make_shared<Box>("side_vbox");
    vbox->SetFlexDirection(Styling::FlexDirection::kColumn);
    vbox->SetJustifyContent(Styling::JustifyContent::kFlexStart);
    vbox->SetAlignItems(Styling::AlignItems::kStretch);
    vbox->SetFlexGrow(0.15f);
    vbox->SetFlexShrink(0.0f);

    // Create information panel
    auto info_panel =
        std::make_shared<PointCloudInfoPanel>("Point Cloud Info", metadata);
    info_panel->SetAutoLayout(true);
    info_panel->SetNoTitleBar(false);  // Make sure title bar is visible
    // info_panel->SetFlexBasis(100.0f);  // Base width
    info_panel->SetFlexGrow(0.38);
    info_panel->SetFlexShrink(0.0f);  // Don't shrink below basis
    info_panel->SetAlwaysAutoResize(false);
    info_panel->SetNoResize(true);
    // info_panel->SetMinWidth(300.0f);   // Set minimum width

    auto tool_panel =
        std::make_shared<PointCloudToolPanel>("Point Cloud Tools", gl_sm.get());
    tool_panel->SetAutoLayout(true);
    tool_panel->SetNoTitleBar(false);  // Make sure title bar is visible
    tool_panel->SetFlexGrow(0.62);
    tool_panel->SetFlexShrink(0.0f);  // Don't shrink below basis
    
    // Connect the tool panel to the scene manager
    gl_sm->SetToolPanel(tool_panel.get());

    vbox->AddChild(info_panel);
    vbox->AddChild(tool_panel);

    // Add components to main container (panel first might help with layout)
    main_box->AddChild(vbox);
    main_box->AddChild(gl_sm);

    // Add to viewer
    viewer.AddSceneObject(main_box);

    std::cout << "Visualization ready. Close the window to exit." << std::endl;
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