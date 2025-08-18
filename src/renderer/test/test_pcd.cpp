/*
 * test_pcd.cpp
 *
 * Created on: Dec 2024
 * Description: Load and visualize PCD (Point Cloud Data) files
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <vector>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "imview/box.hpp"
#include "imview/viewer.hpp"

#include "renderer/gl_scene_manager.hpp"
#include "renderer/renderable/grid.hpp"
#include "renderer/renderable/point_cloud.hpp"

using namespace quickviz;
 
int main(int argc, char* argv[]) {
  // Check command line arguments
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_pcd_file>" << std::endl;
    return 1;
  }

  std::string pcd_file = argv[1];

  // Load PCD file - determine point type from fields
  std::cout << "\n=== Loading PCD File ===" << std::endl;
  std::cout << "File path: " << pcd_file << std::endl;
  
  // First, try to get file info using PCLPointCloud2 for detailed metadata
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile(pcd_file, cloud_blob);
  
  std::cout << "\n=== PCD File Metadata ===" << std::endl;
  std::cout << "Version: " << (cloud_blob.header.seq > 0 ? std::to_string(cloud_blob.header.seq) : "N/A") << std::endl;
  std::cout << "Fields: ";
  for (size_t i = 0; i < cloud_blob.fields.size(); ++i) {
    std::cout << cloud_blob.fields[i].name;
    if (i < cloud_blob.fields.size() - 1) std::cout << ", ";
  }
  std::cout << std::endl;
  std::cout << "Width: " << cloud_blob.width << std::endl;
  std::cout << "Height: " << cloud_blob.height << std::endl;
  std::cout << "Total points: " << cloud_blob.width * cloud_blob.height << std::endl;
  std::cout << "Is dense: " << (cloud_blob.is_dense ? "true" : "false") << std::endl;
  std::cout << "Point step: " << cloud_blob.point_step << " bytes" << std::endl;
  std::cout << "Row step: " << cloud_blob.row_step << " bytes" << std::endl;
  std::cout << "Data size: " << cloud_blob.data.size() << " bytes" << std::endl;
  std::cout << "Is organized: " << (cloud_blob.height > 1 ? "true" : "false") << std::endl;
  
  // Check which fields exist
  bool has_rgb_field = false;
  bool has_intensity_field = false;
  bool has_label_field = false;
  
  for (const auto& field : cloud_blob.fields) {
    if (field.name == "rgb" || field.name == "rgba") {
      has_rgb_field = true;
    }
    if (field.name == "intensity" || field.name == "Intensity" || field.name == "i") {
      has_intensity_field = true;
    }
    if (field.name == "label" || field.name == "Label") {
      has_label_field = true;
    }
  }
  
  std::cout << "\nDetected fields:" << std::endl;
  std::cout << "  RGB: " << (has_rgb_field ? "yes" : "no") << std::endl;
  std::cout << "  Intensity: " << (has_intensity_field ? "yes" : "no") << std::endl;
  std::cout << "  Label: " << (has_label_field ? "yes" : "no") << std::endl;
  
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
      
      std::cout << "RGB colors preserved for true color visualization" << std::endl;
    } else {
      std::cerr << "Warning: RGB field detected but failed to load as XYZRGB" << std::endl;
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

  // Calculate statistics for the loaded points
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  size_t total_points = 0;

  // Calculate statistics from RGB points (3D) or regular points (4D)
  if (has_colors) {
    for (const auto& pt : points_3d) {
      min_x = std::min(min_x, pt.x);
      max_x = std::max(max_x, pt.x);
      min_y = std::min(min_y, pt.y);
      max_y = std::max(max_y, pt.y);
      min_z = std::min(min_z, pt.z);
      max_z = std::max(max_z, pt.z);
      
      sum_x += pt.x;
      sum_y += pt.y;
      sum_z += pt.z;
    }
    total_points = points_3d.size();
  } else {
    for (const auto& pt : points_4d) {
      min_x = std::min(min_x, pt.x);
      max_x = std::max(max_x, pt.x);
      min_y = std::min(min_y, pt.y);
      max_y = std::max(max_y, pt.y);
      min_z = std::min(min_z, pt.z);
      max_z = std::max(max_z, pt.z);
      
      sum_x += pt.x;
      sum_y += pt.y;
      sum_z += pt.z;
    }
    total_points = points_4d.size();
  }

  // Calculate statistics
  float mean_x = sum_x / total_points;
  float mean_y = sum_y / total_points;
  float mean_z = sum_z / total_points;
  
  // Print statistics
  std::cout << "\n=== Point Cloud Statistics ===" << std::endl;
  std::cout << "Total points loaded: " << total_points << std::endl;
  std::cout << "\nSpatial bounds:" << std::endl;
  std::cout << "  X range: [" << min_x << ", " << max_x << "] (width: " << (max_x - min_x) << ")" << std::endl;
  std::cout << "  Y range: [" << min_y << ", " << max_y << "] (depth: " << (max_y - min_y) << ")" << std::endl;
  std::cout << "  Z range: [" << min_z << ", " << max_z << "] (height: " << (max_z - min_z) << ")" << std::endl;
  std::cout << "\nCentroid:" << std::endl;
  std::cout << "  X: " << mean_x << std::endl;
  std::cout << "  Y: " << mean_y << std::endl;
  std::cout << "  Z: " << mean_z << std::endl;
  
  if (has_colors) {
    std::cout << "\nColor information:" << std::endl;
    std::cout << "  RGB colors available: yes (" << colors_rgb.size() << " colors)" << std::endl;
  } else if (has_intensity) {
    std::cout << "\nIntensity information:" << std::endl;
    std::cout << "  Intensity values normalized to [0, 1]" << std::endl;
  }

  std::cout << "\n=== Visualization Info ===" << std::endl;
  std::cout << "Points to render: " << total_points << std::endl;
  if (has_colors) {
    std::cout << "Color mode: True RGB colors" << std::endl;
  } else if (has_intensity) {
    std::cout << "Color mode: Intensity field" << std::endl;
  } else {
    std::cout << "Color mode: Height field (by Z value)" << std::endl;
  }

  // Create viewer
  Viewer viewer;

  // create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // create a OpenGL scene manager to manage the OpenGL objects
  auto gl_sm = std::make_shared<GlSceneManager>("OpenGL Scene");
  gl_sm->SetAutoLayout(true);
  gl_sm->SetNoTitleBar(true);
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
    std::cout << "\nUsing true RGB color visualization" << std::endl;
  } else if (has_intensity) {
    // Use intensity/scalar field (w component already normalized)
    point_cloud->SetScalarRange(0.0f, 1.0f);
    point_cloud->SetPoints(points_4d, PointCloud::ColorMode::kScalarField);
    std::cout << "\nUsing intensity field visualization" << std::endl;
  } else {
    // Use height field
    point_cloud->SetScalarRange(min_z, max_z);
    point_cloud->SetPoints(points_4d, PointCloud::ColorMode::kHeightField);
    std::cout << "\nUsing height field visualization" << std::endl;
  }

  // Add point cloud to the scene manager
  gl_sm->AddOpenGLObject("point_cloud", std::move(point_cloud));
  
  // Add a grid for reference
  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // finally pass the OpenGL scene manager to the box and add it to the viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}