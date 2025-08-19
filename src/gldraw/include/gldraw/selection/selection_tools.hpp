/**
 * @file selection_tools.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2024-12-19
 * @brief Interactive selection tools for point clouds
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SELECTION_TOOLS_HPP
#define QUICKVIZ_SELECTION_TOOLS_HPP

#include <memory>
#include <vector>
#include <functional>
#include <glm/glm.hpp>

#include "gldraw/selection/selection_result.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/camera.hpp"

namespace quickviz {

/**
 * @brief Interactive selection tools for point clouds in screen space
 */
class SelectionTools {
 public:
  SelectionTools();
  ~SelectionTools();
  
  // Set the point cloud and camera for selection operations
  void SetPointCloud(PointCloud* point_cloud);
  void SetCamera(const Camera* camera);
  void SetViewport(int x, int y, int width, int height);
  void SetProjectionMatrix(const glm::mat4& projection);
  void SetViewMatrix(const glm::mat4& view);
  void SetCoordinateTransform(const glm::mat4& transform);
  
  // Selection methods
  
  /**
   * @brief Pick a single point at screen position
   * @param screen_x Screen X coordinate
   * @param screen_y Screen Y coordinate  
   * @param tolerance Pixel tolerance for selection (default: 5 pixels)
   * @return Index of selected point, or -1 if no point selected
   */
  int PickPoint(float screen_x, float screen_y, float tolerance = 5.0f);
  
  /**
   * @brief Select points within a rectangle
   * @param x1, y1 First corner of rectangle in screen coordinates
   * @param x2, y2 Second corner of rectangle in screen coordinates
   * @param mode Selection mode (replace, add, remove, toggle)
   * @return Selection result with selected point indices
   */
  SelectionResult SelectRectangle(float x1, float y1, float x2, float y2,
                                  SelectionMode mode = SelectionMode::kReplace);
  
  /**
   * @brief Select points within a lasso/polygon region
   * @param screen_points Polygon vertices in screen coordinates
   * @param mode Selection mode (replace, add, remove, toggle)
   * @return Selection result with selected point indices
   */
  SelectionResult SelectLasso(const std::vector<glm::vec2>& screen_points,
                             SelectionMode mode = SelectionMode::kReplace);
  
  /**
   * @brief Select points within a radius of a screen position
   * @param center_x, center_y Center position in screen coordinates
   * @param radius Radius in pixels
   * @param mode Selection mode (replace, add, remove, toggle)
   * @return Selection result with selected point indices
   */
  SelectionResult SelectRadius(float center_x, float center_y, float radius,
                              SelectionMode mode = SelectionMode::kReplace);
  
  /**
   * @brief Grow selection to include neighboring points
   * @param distance_threshold Maximum distance to include neighbors
   */
  void GrowSelection(float distance_threshold);
  
  /**
   * @brief Shrink selection by removing boundary points
   * @param distance_threshold Minimum distance from selection boundary
   */
  void ShrinkSelection(float distance_threshold);
  
  // Selection state management
  const SelectionResult& GetCurrentSelection() const { return current_selection_; }
  void ClearSelection();
  void InvertSelection();
  
  // Callbacks
  void SetSelectionCallback(SelectionCallback callback) {
    selection_callback_ = std::move(callback);
  }
  void SetHoverCallback(HoverCallback callback) {
    hover_callback_ = std::move(callback);
  }
  
  // Utility methods
  
  /**
   * @brief Convert screen coordinates to normalized device coordinates
   * @param screen_x, screen_y Screen coordinates
   * @return NDC coordinates (-1 to 1)
   */
  glm::vec2 ScreenToNDC(float screen_x, float screen_y) const;
  
  /**
   * @brief Project 3D point to screen coordinates
   * @param world_point 3D point in world space
   * @return Screen coordinates
   */
  glm::vec2 WorldToScreen(const glm::vec3& world_point) const;
  
  /**
   * @brief Check if a point is inside a 2D polygon
   * @param point Point to test
   * @param polygon Polygon vertices
   * @return true if point is inside polygon
   */
  static bool IsPointInPolygon(const glm::vec2& point,
                               const std::vector<glm::vec2>& polygon);
  
  /**
   * @brief Check if a point is inside a rectangle
   * @param point Point to test
   * @param min_corner Minimum corner of rectangle
   * @param max_corner Maximum corner of rectangle
   * @return true if point is inside rectangle
   */
  static bool IsPointInRectangle(const glm::vec2& point,
                                 const glm::vec2& min_corner,
                                 const glm::vec2& max_corner);
  
  // Hover support
  void UpdateHover(float screen_x, float screen_y);
  int GetHoveredPoint() const { return hovered_point_index_; }
  
 private:
  // Helper methods
  void UpdateSelectionResult(const std::vector<size_t>& indices,
                            SelectionResult::Method method);
  std::vector<size_t> ApplySelectionMode(const std::vector<size_t>& new_indices,
                                         SelectionMode mode);
  float ComputeScreenDistance(const glm::vec3& world_point,
                             float screen_x, float screen_y) const;
  
  // References to external objects
  PointCloud* point_cloud_ = nullptr;
  const Camera* camera_ = nullptr;
  
  // Viewport and matrices
  glm::ivec4 viewport_;  // x, y, width, height
  glm::mat4 projection_matrix_;
  glm::mat4 view_matrix_;
  glm::mat4 coord_transform_;
  
  // Selection state
  SelectionResult current_selection_;
  int hovered_point_index_ = -1;
  
  // Callbacks
  SelectionCallback selection_callback_;
  HoverCallback hover_callback_;
  
  // Performance optimization
  mutable std::vector<glm::vec2> screen_cache_;  // Cached screen positions
  mutable bool screen_cache_valid_ = false;
  void InvalidateScreenCache() { screen_cache_valid_ = false; }
  void UpdateScreenCache() const;
};

}  // namespace quickviz

#endif  // QUICKVIZ_SELECTION_TOOLS_HPP