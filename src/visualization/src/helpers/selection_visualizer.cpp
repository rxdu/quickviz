/*
 * @file selection_visualizer.cpp
 * @date 2025-08-23
 * @brief Implementation of SelectionVisualizer class
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "visualization/helpers/selection_visualizer.hpp"
#include <sstream>
#include <chrono>
#include <iostream>

namespace quickviz {
namespace visualization {

bool SelectionVisualizer::CreateHighlight(const SelectionData& selection_data, PointCloud& target_cloud) {
  if (selection_data.IsEmpty()) {
    std::cerr << "SelectionVisualizer::CreateHighlight: Empty selection data" << std::endl;
    return false;
  }
  
  if (target_cloud.GetPointCount() == 0) {
    std::cerr << "SelectionVisualizer::CreateHighlight: Target point cloud is empty" << std::endl;
    return false;
  }
  
  // Validate indices are within bounds
  for (size_t idx : selection_data.point_indices) {
    if (idx >= target_cloud.GetPointCount()) {
      std::cerr << "SelectionVisualizer::CreateHighlight: Invalid point index " 
                << idx << " (cloud has " << target_cloud.GetPointCount() << " points)" << std::endl;
      return false;
    }
  }
  
  // Generate layer name if needed
  std::string layer_name = selection_data.selection_name;
  if (layer_name.empty() || layer_name == "selection") {
    layer_name = GenerateLayerName(selection_data.selection_name);
  }
  
  // Create highlight layer with high priority
  auto layer = target_cloud.CreateLayer(layer_name, 100);
  if (!layer) {
    std::cerr << "SelectionVisualizer::CreateHighlight: Failed to create layer '" 
              << layer_name << "'" << std::endl;
    return false;
  }
  
  // Configure the layer with selection data
  layer->SetPoints(selection_data.point_indices);
  layer->SetColor(selection_data.highlight_color);
  layer->SetPointSizeMultiplier(selection_data.size_multiplier);
  layer->SetHighlightMode(PointLayer::HighlightMode::kSphereSurface);
  layer->SetVisible(true);
  
  std::cout << "SelectionVisualizer: Created highlight layer '" << layer_name 
            << "' with " << selection_data.point_indices.size() << " points" << std::endl;
  
  return true;
}

bool SelectionVisualizer::CreateHighlight(const std::vector<size_t>& point_indices,
                                         const glm::vec3& color,
                                         PointCloud& target_cloud,
                                         const std::string& layer_name,
                                         float size_multiplier) {
  // Create SelectionData from parameters
  SelectionData selection_data;
  selection_data.point_indices = point_indices;
  selection_data.highlight_color = color;
  selection_data.size_multiplier = size_multiplier;
  selection_data.selection_name = layer_name.empty() ? "selection" : layer_name;
  
  return CreateHighlight(selection_data, target_cloud);
}

bool SelectionVisualizer::RemoveHighlight(const std::string& layer_name, PointCloud& target_cloud) {
  if (layer_name.empty()) {
    std::cerr << "SelectionVisualizer::RemoveHighlight: Empty layer name" << std::endl;
    return false;
  }
  
  bool success = target_cloud.RemoveLayer(layer_name);
  if (success) {
    std::cout << "SelectionVisualizer: Removed highlight layer '" << layer_name << "'" << std::endl;
  } else {
    std::cerr << "SelectionVisualizer::RemoveHighlight: Layer '" << layer_name << "' not found" << std::endl;
  }
  
  return success;
}

size_t SelectionVisualizer::ClearAllHighlights(PointCloud& target_cloud) {
  // Use the existing ClearAllLayers method from PointCloud
  target_cloud.ClearAllLayers();
  
  std::cout << "SelectionVisualizer: Cleared all highlight layers" << std::endl;
  
  return 1; // Return success indicator since we don't track the count
}

std::string SelectionVisualizer::GenerateLayerName(const std::string& base_name) {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  
  std::ostringstream oss;
  oss << base_name << "_" << time_t << "_" << ms.count();
  return oss.str();
}

}  // namespace visualization  
}  // namespace quickviz