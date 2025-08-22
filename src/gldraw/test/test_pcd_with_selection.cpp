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
#include "visualization/contracts/selection_data.hpp"
#include "visualization/renderables/selection_renderable.hpp"
#include "visualization/testing/mock_data_generator.hpp"
#include "visualization/pcl_bridge/pcl_loader.hpp"

using namespace quickviz;

// Demo panel showing external selection processing
class ExternalSelectionDemoPanel : public Panel {
 public:
  ExternalSelectionDemoPanel(const pcl_bridge::PointCloudMetadata& metadata) 
      : Panel("External Selection Demo"), metadata_(metadata) {
    SetAutoLayout(true);
  }
  
  void Draw() override {
    ImGui::Text("External Selection Demo");
    ImGui::Separator();
    
    // File info
    ImGui::Text("File: %s", std::filesystem::path(metadata_.filename).filename().c_str());
    ImGui::Text("Points: %zu", metadata_.point_count);
    ImGui::Text("Format: %s", metadata_.format.c_str());
    ImGui::Text("PCL Type: %s", metadata_.detected_pcl_type.c_str());
    ImGui::Separator();
    
    // Demo selections
    ImGui::Text("Demo Selection Types:");
    if (ImGui::Button("Random Selection (100 pts)", ImVec2(200, 0))) {
      CreateRandomSelection();
    }
    if (ImGui::Button("Rectangle Selection", ImVec2(200, 0))) {
      CreateRectangleSelection();
    }
    if (ImGui::Button("Multiple Clusters", ImVec2(200, 0))) {
      CreateMultipleSelections();
    }
    if (ImGui::Button("Clear All Selections", ImVec2(200, 0))) {
      ClearAllSelections();
    }
    
    ImGui::Separator();
    
    // Algorithm simulation
    ImGui::Text("Simulated Processing:");
    ImGui::SliderFloat("Confidence", &confidence_, 0.0f, 1.0f);
    if (ImGui::Button("Simulate Segmentation", ImVec2(200, 0))) {
      SimulateSegmentation();
    }
    if (ImGui::Button("Simulate Feature Detection", ImVec2(200, 0))) {
      SimulateFeatureDetection();
    }
    
    ImGui::Separator();
    
    // Selection statistics
    ImGui::Text("Current Selections: %zu", active_selections_.size());
    if (!active_selections_.empty()) {
      ImGui::Text("Recent Selection Info:");
      const auto& last_selection = active_selections_.back();
      ImGui::Text("  Name: %s", last_selection.selection_name.c_str());
      ImGui::Text("  Points: %zu", last_selection.point_indices.size());
      ImGui::Text("  Color: (%.2f, %.2f, %.2f)", 
                  last_selection.highlight_color.r,
                  last_selection.highlight_color.g,
                  last_selection.highlight_color.b);
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
    ImGui::TextWrapped("This demo shows how external processing");
    ImGui::TextWrapped("algorithms would provide selection results");
    ImGui::TextWrapped("to gldraw for visualization.");
    ImGui::Separator();
    ImGui::TextWrapped("Camera controls:");
    ImGui::TextWrapped("• Right click: rotate");
    ImGui::TextWrapped("• Scroll: zoom");
    ImGui::TextWrapped("• Middle click: pan");
  }
  
  // Remove visualizer setter since we'll use static methods
  void SetPointCloud(PointCloud* pc) { point_cloud_ = pc; }
  
 private:
  void CreateRandomSelection() {
    auto selection = visualization::testing::MockDataGenerator::GenerateRandomSelection(
        metadata_.point_count, 0.05f); // 5% of points
    selection.selection_name = "random_demo";
    selection.highlight_color = glm::vec3(1.0f, 0.8f, 0.2f); // Orange
    VisualizeSelection(selection);
  }
  
  void CreateRectangleSelection() {
    auto selection = visualization::testing::MockDataGenerator::GenerateRectangularSelection(
        metadata_.point_count, 0, 100); // First 100 points
    selection.selection_name = "rectangle_demo";
    selection.highlight_color = glm::vec3(0.2f, 0.8f, 1.0f); // Cyan
    VisualizeSelection(selection);
  }
  
  void CreateMultipleSelections() {
    auto selections = visualization::testing::MockDataGenerator::GenerateMultipleSelections(
        metadata_.point_count, 3);
    glm::vec3 colors[] = {
        glm::vec3(1.0f, 0.2f, 0.2f), // Red
        glm::vec3(0.2f, 1.0f, 0.2f), // Green
        glm::vec3(0.2f, 0.2f, 1.0f)  // Blue
    };
    
    for (size_t i = 0; i < selections.size(); ++i) {
      selections[i].selection_name = "cluster_" + std::to_string(i);
      selections[i].highlight_color = colors[i % 3];
      VisualizeSelection(selections[i]);
    }
  }
  
  void SimulateSegmentation() {
    auto selection = visualization::testing::MockDataGenerator::GenerateRandomSelection(
        metadata_.point_count, 0.1f); // 10% of points
    selection.selection_name = "segmentation_result";
    // Confidence affects color intensity
    selection.highlight_color = glm::vec3(confidence_, 1.0f, 0.3f);
    selection.size_multiplier = 1.0f + confidence_ * 0.5f;
    VisualizeSelection(selection);
  }
  
  void SimulateFeatureDetection() {
    auto selection = visualization::testing::MockDataGenerator::GenerateRandomSelection(
        metadata_.point_count, 0.03f); // 3% of points
    selection.selection_name = "feature_detection";
    // High confidence features get brighter, larger points
    selection.highlight_color = glm::vec3(1.0f, 0.3f, confidence_);
    selection.size_multiplier = 1.5f + confidence_;
    VisualizeSelection(selection);
  }
  
  void VisualizeSelection(const visualization::SelectionData& selection) {
    if (point_cloud_) {
      // Create a SelectionRenderable using the new API
      auto renderable = visualization::SelectionRenderable::FromData(selection, *point_cloud_);
      if (renderable) {
        renderable->AllocateGpuResources();
        active_renderables_.push_back(std::move(renderable));
        active_selections_.push_back(selection);
        
        // Limit to last 5 selections to avoid clutter
        if (active_selections_.size() > 5) {
          // Remove the oldest selection
          active_renderables_.erase(active_renderables_.begin());
          active_selections_.erase(active_selections_.begin());
        }
      }
    }
  }
  
  void ClearAllSelections() {
    // Clear all active renderable objects (they will clean up automatically)
    active_renderables_.clear();
    active_selections_.clear();
  }
  
  PointCloud* point_cloud_ = nullptr;
  
  pcl_bridge::PointCloudMetadata metadata_;
  std::vector<visualization::SelectionData> active_selections_;
  std::vector<std::unique_ptr<visualization::SelectionRenderable>> active_renderables_;
  
  float confidence_ = 0.8f;
  float point_size_ = 2.0f;
  float opacity_ = 1.0f;
};

// Simple scene manager for external selection demo
class ExternalSelectionSceneManager : public GlSceneManager {
 public:
  ExternalSelectionSceneManager() : GlSceneManager("External Selection Demo") {
    SetShowRenderingInfo(true);
    SetBackgroundColor(0.05f, 0.05f, 0.08f, 1.0f);
    SetNoTitleBar(true);
  }
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

    // Create scene manager for external selection demo
    auto gl_sm = std::make_shared<ExternalSelectionSceneManager>();
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

    // Create demo control panel
    auto selection_panel = std::make_shared<ExternalSelectionDemoPanel>(metadata);
    selection_panel->SetPointCloud(pc_ptr);
    selection_panel->SetAutoLayout(true);
    selection_panel->SetFlexGrow(0.15f);     // Panel takes less space
    selection_panel->SetFlexShrink(0.0f);   // Don't shrink below basis

    // Add components to main container
    main_box->AddChild(selection_panel);
    main_box->AddChild(gl_sm);
    
    // Add to viewer
    viewer.AddSceneObject(main_box);

    std::cout << "\n=== Starting External Selection Demo ===" << std::endl;
    std::cout << "Use the left panel to generate demo selections from external processing" << std::endl;
    std::cout << "This demonstrates how external algorithms would provide results to gldraw" << std::endl;
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