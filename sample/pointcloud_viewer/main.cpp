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
#include "visualization/pcl_bridge/pcl_loader.hpp"

#include "point_cloud_info_panel.hpp"
#include "point_cloud_tool_panel.hpp"
#include "interactive_scene_manager.hpp"

using namespace quickviz;
using namespace quickviz::pcl_bridge;

void AnalyzePcMetaData(const std::string& filename) {
  std::cout << "\n=== Analyzing Point Cloud File ===" << std::endl;
  auto analysis_metadata =
      pcl_bridge::PointCloudLoader::AnalyzeFields(filename);

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
}

void PrintPcMetaData(const PointCloudMetadata& metadata) {
  std::cout << "Factory loading completed successfully!" << std::endl;

  std::cout << "\n=== Load Results ===" << std::endl;
  std::cout << "Successfully loaded " << metadata.point_count << " points"
            << std::endl;
  std::cout << "Detected PCL type: " << metadata.detected_pcl_type << std::endl;
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
    AnalyzePcMetaData(point_cloud_file);

    // Load point cloud using the PCL loader with progress callback
    std::cout << "\n=== Loading Point Cloud ===" << std::endl;

    // Try without callback first to isolate the issue
    // auto progress_callback = [](float progress, const std::string& message) {
    //   std::cout << "Progress: " << static_cast<int>(progress * 100) << "% - "
    //   << message << std::endl;
    // };

    // Load point cloud using the new factory pattern
    std::cout << "\n=== Loading with Factory Pattern ===" << std::endl;
    std::cout << "Using FactoryRegistry to load renderer-optimized data..."
              << std::endl;

    auto [renderer_data, metadata] =
        pcl_bridge::factory::FactoryRegistry::LoadForRenderer(
            point_cloud_file,
            pcl_bridge::PointCloudLoader::Format::kAutoDetect);

    PrintPcMetaData(metadata);

    ///////////////////////////////////////////////////////////////////////////

    // Create viewer for visualization (this initializes OpenGL context)
    std::cout << "\n=== Creating Visualization ===" << std::endl;
    Viewer viewer;

    // Create main container box
    auto main_box = std::make_shared<Box>("main_container");
    main_box->SetFlexDirection(Styling::FlexDirection::kRow);
    main_box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
    main_box->SetAlignItems(Styling::AlignItems::kStretch);

    // Create OpenGL scene manager for 3D visualization
    auto gl_sm =
        std::make_shared<InteractiveSceneManager>("Point Cloud Viewer");
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

    // Configure point cloud based on factory output
    std::cout << "Configuring point cloud with "
              << renderer_data.GetPointCount() << " points..." << std::endl;

    switch (renderer_data.color_mode) {
      case pcl_bridge::factory::RendererData::ColorMode::kRGB:
        std::cout << "Using RGB coloring" << std::endl;
        point_cloud->SetPoints(renderer_data.points_3d,
                               renderer_data.colors_rgb);
        break;

      case pcl_bridge::factory::RendererData::ColorMode::kIntensity:
        std::cout << "Using intensity coloring" << std::endl;
        point_cloud->SetScalarRange(renderer_data.scalar_range.x,
                                    renderer_data.scalar_range.y);
        point_cloud->SetPoints(renderer_data.points_4d,
                               PointCloud::ColorMode::kScalarField);
        break;

      case pcl_bridge::factory::RendererData::ColorMode::kHeight:
        std::cout << "Using height-based coloring" << std::endl;
        point_cloud->SetScalarRange(renderer_data.scalar_range.x,
                                    renderer_data.scalar_range.y);
        point_cloud->SetPoints(renderer_data.points_4d,
                               PointCloud::ColorMode::kHeightField);
        break;

      case pcl_bridge::factory::RendererData::ColorMode::kScalar:
        std::cout << "Using scalar field coloring" << std::endl;
        point_cloud->SetScalarRange(renderer_data.scalar_range.x,
                                    renderer_data.scalar_range.y);
        point_cloud->SetPoints(renderer_data.points_4d,
                               PointCloud::ColorMode::kScalarField);
        break;
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