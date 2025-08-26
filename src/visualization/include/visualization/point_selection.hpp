/**
 * @file point_selection.hpp  
 * @date 2025-08-26
 * @brief Simplified point selection utilities for interactive point cloud workflows
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef VISUALIZATION_POINT_SELECTION_HPP
#define VISUALIZATION_POINT_SELECTION_HPP

#include <memory>
#include <vector>
#include <functional>
#include <optional>

#include <glm/glm.hpp>
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/gl_scene_manager.hpp"

namespace quickviz {
namespace visualization {

/**
 * @brief Simplified point selection for interactive point cloud workflows
 * 
 * This class provides convenient point-by-point selection using existing GPU picking,
 * with easy access to selected point data for external PCL processing.
 */
class PointSelection {
public:
  /**
   * @brief Constructor
   * @param cloud Point cloud to operate on
   * @param scene_manager Scene manager for GPU picking operations
   */
  PointSelection(std::shared_ptr<PointCloud> cloud, GlSceneManager* scene_manager);

  // === Selection Operations ===
  
  /**
   * @brief Select a single point (replace current selection)
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate  
   * @param radius Picking radius in pixels
   * @return true if a point was selected
   */
  bool SelectPoint(float screen_x, float screen_y, int radius = 3);
  
  /**
   * @brief Add a point to current selection
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate
   * @param radius Picking radius in pixels
   * @return true if a point was selected and added
   */
  bool AddPoint(float screen_x, float screen_y, int radius = 3);
  
  /**
   * @brief Toggle point selection state
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate
   * @param radius Picking radius in pixels
   * @return true if a point was found and toggled
   */
  bool TogglePoint(float screen_x, float screen_y, int radius = 3);
  
  /**
   * @brief Clear all selected points
   */
  void ClearSelection();

  // === Selection State ===
  
  /**
   * @brief Get indices of selected points
   * @return Vector of point indices in the original point cloud
   */
  const std::vector<size_t>& GetSelectedIndices() const { return selected_indices_; }
  
  /**
   * @brief Get number of selected points
   */
  size_t GetSelectionCount() const { return selected_indices_.size(); }

  // === Point Data Access (for PCL processing) ===
  
  /**
   * @brief Get 3D positions of selected points
   * @return Vector of 3D positions suitable for PCL processing
   */
  std::vector<glm::vec3> GetSelectedPoints() const;
  
  /**
   * @brief Get colors of selected points (if available)
   * @return Vector of colors, or empty if point cloud has no color data
   */
  std::vector<glm::vec3> GetSelectedColors() const;
  
  /**
   * @brief Get a single selected point by index
   * @param selection_index Index within the selection (not the original cloud index)
   * @return 3D position if valid index, nullopt otherwise
   */
  std::optional<glm::vec3> GetSelectedPoint(size_t selection_index) const;

  // === Convenience Statistics ===
  
  /**
   * @brief Get centroid of selected points
   * @return Centroid position, or zero vector if no selection
   */
  glm::vec3 GetSelectionCentroid() const;
  
  /**
   * @brief Get bounding box of selected points  
   * @return {min_bounds, max_bounds} or {{0,0,0}, {0,0,0}} if no selection
   */
  std::pair<glm::vec3, glm::vec3> GetSelectionBounds() const;

  // === Visualization Control ===
  
  /**
   * @brief Configure selection visualization
   * @param color Highlight color (default: yellow)
   * @param size_multiplier Point size multiplier (default: 1.5x)
   * @param layer_name Layer name for highlights (default: "selection")
   */
  void SetSelectionVisualization(const glm::vec3& color = glm::vec3(1.0f, 1.0f, 0.0f),
                                float size_multiplier = 1.5f,
                                const std::string& layer_name = "selection");
  
  /**
   * @brief Enable/disable selection visualization
   * @param enabled Whether to show selection highlights
   */
  void SetVisualizationEnabled(bool enabled);

  // === Callbacks ===
  
  /**
   * @brief Callback type for selection changes
   * @param indices Currently selected point indices
   */
  using SelectionCallback = std::function<void(const std::vector<size_t>&)>;
  
  /**
   * @brief Set callback for selection changes
   * @param callback Function to call when selection changes
   */
  void SetSelectionCallback(SelectionCallback callback) {
    callback_ = callback;
  }

private:
  // Core components
  std::shared_ptr<PointCloud> point_cloud_;
  GlSceneManager* scene_manager_;
  
  // Selection state
  std::vector<size_t> selected_indices_;
  SelectionCallback callback_;
  
  // Visualization settings
  glm::vec3 highlight_color_ = glm::vec3(1.0f, 1.0f, 0.0f);  // Yellow
  float size_multiplier_ = 1.5f;
  std::string layer_name_ = "selection";
  bool visualization_enabled_ = true;
  
  // Internal methods
  std::optional<size_t> PickPointAtScreenPos(float x, float y, int radius);
  void UpdateVisualization();
  void NotifySelectionChanged();
  bool IsPointSelected(size_t point_index) const;
  void RemoveFromSelection(size_t point_index);
  void AddToSelection(size_t point_index);
};

}  // namespace visualization
}  // namespace quickviz

#endif  // VISUALIZATION_POINT_SELECTION_HPP