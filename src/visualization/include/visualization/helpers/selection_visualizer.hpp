/*
 * @file selection_visualizer.hpp
 * @date 2025-08-23
 * @brief High-level visualizer for point cloud selections
 *
 * This provides convenient static methods for creating selection highlights
 * without needing to manage renderables directly.
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef VISUALIZATION_SELECTION_VISUALIZER_HPP
#define VISUALIZATION_SELECTION_VISUALIZER_HPP

#include "visualization/contracts/selection_data.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include <memory>
#include <string>

namespace quickviz {
namespace visualization {

/**
 * @brief High-level visualizer for point cloud selections
 * 
 * This class provides convenient static methods for creating selection highlights
 * directly on PointCloud objects without needing to manage SelectionRenderable objects.
 */
class SelectionVisualizer {
public:
  /**
   * @brief Create a highlight directly on a point cloud
   * @param selection_data Selection specification from external processing
   * @param target_cloud Point cloud to highlight
   * @return Success status
   */
  static bool CreateHighlight(const SelectionData& selection_data, PointCloud& target_cloud);
  
  /**
   * @brief Create a highlight with simple parameters
   * @param point_indices Indices of points to highlight
   * @param color Highlight color
   * @param target_cloud Point cloud to highlight
   * @param layer_name Optional layer name (auto-generated if empty)
   * @param size_multiplier Point size multiplier for highlighting
   * @return Success status
   */
  static bool CreateHighlight(const std::vector<size_t>& point_indices,
                             const glm::vec3& color,
                             PointCloud& target_cloud,
                             const std::string& layer_name = "",
                             float size_multiplier = 1.5f);
  
  /**
   * @brief Remove a highlight layer
   * @param layer_name Name of the layer to remove
   * @param target_cloud Point cloud containing the layer
   * @return Success status
   */
  static bool RemoveHighlight(const std::string& layer_name, PointCloud& target_cloud);
  
  /**
   * @brief Clear all selection highlights from a point cloud
   * @param target_cloud Point cloud to clear
   * @return Number of layers removed
   */
  static size_t ClearAllHighlights(PointCloud& target_cloud);

private:
  /**
   * @brief Generate a unique layer name
   * @param base_name Base name for the layer
   * @return Unique layer name with timestamp
   */
  static std::string GenerateLayerName(const std::string& base_name = "selection");
};

}  // namespace visualization
}  // namespace quickviz

#endif  // VISUALIZATION_SELECTION_VISUALIZER_HPP