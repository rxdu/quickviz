/**
 * @file point_cloud_selector.hpp
 * @date 2025-08-25
 * @brief Point cloud selection and picking utilities using PCL for spatial queries
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef VISUALIZATION_SELECTION_POINT_CLOUD_SELECTOR_HPP
#define VISUALIZATION_SELECTION_POINT_CLOUD_SELECTOR_HPP

#include <memory>
#include <vector>
#include <optional>
#include <functional>

#include <glm/glm.hpp>
#include "gldraw/renderable/point_cloud.hpp"

// Forward declarations for PCL types  
namespace pcl {
template<typename PointT>
class PointCloud;
struct PointXYZ;
}

// Need to include KdTreeFLANN for proper template instantiation
#ifdef QUICKVIZ_WITH_PCL
#include <pcl/kdtree/kdtree_flann.h>
#else
// Provide a dummy declaration when PCL is not available
namespace pcl {
template<typename PointT>
class KdTreeFLANN;
}
#endif

namespace quickviz {
namespace visualization {

/**
 * @brief Ray structure for 3D picking
 */
struct Ray {
  glm::vec3 origin;
  glm::vec3 direction;  // Should be normalized
  
  Ray(const glm::vec3& o, const glm::vec3& d) 
    : origin(o), direction(glm::normalize(d)) {}
};

/**
 * @brief Result of a point picking operation
 */
struct PickResult {
  size_t point_index;      // Index of the picked point
  glm::vec3 point;         // 3D position of the picked point
  float distance;          // Distance from ray origin to point
  glm::vec3 screen_point;  // Projected screen coordinates (if available)
};

/**
 * @brief Selection modes for point cloud operations
 */
enum class SelectionMode {
  kSingle,      // Select single point (replace current)
  kAdditive,    // Add to current selection
  kSubtractive, // Remove from current selection
  kToggle       // Toggle selection state
};

/**
 * @brief Selection region types
 */
enum class SelectionRegion {
  kSphere,      // Spherical region around a point
  kBox,         // Axis-aligned bounding box
  kCylinder,    // Cylindrical region
  kCone,        // Conical region from ray
  kPlane        // Points on one side of a plane
};

/**
 * @brief Point cloud selector for interactive selection and picking
 * 
 * This class provides high-level selection operations for point clouds,
 * using PCL's spatial data structures for efficient queries.
 */
class PointCloudSelector {
 public:
  PointCloudSelector();
  ~PointCloudSelector();

  /**
   * @brief Set the point cloud for selection operations
   * @param point_cloud The renderer point cloud to operate on
   */
  void SetPointCloud(std::shared_ptr<PointCloud> point_cloud);

  /**
   * @brief Update internal spatial index (call after point cloud changes)
   */
  void UpdateSpatialIndex();

  // --- Point Picking Operations ---
  
  /**
   * @brief Pick a single point using ray casting
   * @param ray The picking ray in world coordinates
   * @param max_distance Maximum distance from ray to consider points
   * @return The pick result if a point was found
   */
  std::optional<PickResult> PickPoint(const Ray& ray, 
                                      float max_distance = 0.1f) const;

  /**
   * @brief Pick the nearest point to a 3D position
   * @param position The 3D position in world coordinates
   * @param max_distance Maximum distance to search
   * @return The pick result if a point was found
   */
  std::optional<PickResult> PickNearestPoint(const glm::vec3& position,
                                             float max_distance = 1.0f) const;

  /**
   * @brief Pick multiple points along a ray
   * @param ray The picking ray
   * @param max_distance Maximum distance from ray
   * @param max_points Maximum number of points to return
   * @return Vector of pick results, sorted by distance
   */
  std::vector<PickResult> PickPointsAlongRay(const Ray& ray,
                                             float max_distance = 0.1f,
                                             size_t max_points = 10) const;

  // --- Region Selection Operations ---

  /**
   * @brief Select points within a sphere
   * @param center Sphere center
   * @param radius Sphere radius
   * @return Indices of selected points
   */
  std::vector<size_t> SelectInSphere(const glm::vec3& center, 
                                     float radius) const;

  /**
   * @brief Select points within an axis-aligned bounding box
   * @param min Minimum corner of the box
   * @param max Maximum corner of the box
   * @return Indices of selected points
   */
  std::vector<size_t> SelectInBox(const glm::vec3& min, 
                                  const glm::vec3& max) const;

  /**
   * @brief Select points on one side of a plane
   * @param point Point on the plane
   * @param normal Plane normal (pointing to the positive side)
   * @param select_positive If true, select points on positive side
   * @return Indices of selected points
   */
  std::vector<size_t> SelectByPlane(const glm::vec3& point,
                                    const glm::vec3& normal,
                                    bool select_positive = true) const;

  /**
   * @brief Select points within a cylinder
   * @param base Base center of cylinder
   * @param axis Cylinder axis (normalized)
   * @param height Cylinder height
   * @param radius Cylinder radius
   * @return Indices of selected points
   */
  std::vector<size_t> SelectInCylinder(const glm::vec3& base,
                                       const glm::vec3& axis,
                                       float height,
                                       float radius) const;

  // --- Selection State Management ---

  /**
   * @brief Get currently selected point indices
   */
  const std::vector<size_t>& GetSelectedIndices() const { 
    return selected_indices_; 
  }

  /**
   * @brief Set selected points directly
   */
  void SetSelectedIndices(const std::vector<size_t>& indices);

  /**
   * @brief Clear all selections
   */
  void ClearSelection();

  /**
   * @brief Update selection with new indices based on mode
   * @param indices New point indices to process
   * @param mode Selection mode (single, additive, etc.)
   */
  void UpdateSelection(const std::vector<size_t>& indices,
                      SelectionMode mode = SelectionMode::kSingle);

  /**
   * @brief Apply selection visualization to the point cloud
   * @param layer_name Name of the visualization layer
   * @param color Highlight color
   * @param size_multiplier Point size multiplier for highlights
   */
  void ApplySelectionVisualization(const std::string& layer_name = "selection",
                                   const glm::vec3& color = glm::vec3(1.0f, 1.0f, 0.0f),
                                   float size_multiplier = 1.5f);

  // --- Statistics and Analysis ---

  /**
   * @brief Get bounding box of selected points
   */
  std::pair<glm::vec3, glm::vec3> GetSelectionBounds() const;

  /**
   * @brief Get centroid of selected points
   */
  glm::vec3 GetSelectionCentroid() const;

  /**
   * @brief Get number of selected points
   */
  size_t GetSelectionCount() const { return selected_indices_.size(); }

  // --- Callbacks ---

  /**
   * @brief Set callback for selection changes
   */
  using SelectionCallback = std::function<void(const std::vector<size_t>&)>;
  void SetSelectionCallback(SelectionCallback callback) {
    selection_callback_ = callback;
  }

 private:
  // Point cloud data
  std::shared_ptr<PointCloud> point_cloud_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud_;
  std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> kdtree_;

  // Selection state
  std::vector<size_t> selected_indices_;
  SelectionCallback selection_callback_;

  // Helper methods
  void BuildPCLCloud();
  void NotifySelectionChanged();
  float PointToRayDistance(const glm::vec3& point, const Ray& ray) const;
  bool IsPointInBox(const glm::vec3& point, 
                   const glm::vec3& min, 
                   const glm::vec3& max) const;
  bool IsPointInCylinder(const glm::vec3& point,
                        const glm::vec3& base,
                        const glm::vec3& axis,
                        float height,
                        float radius) const;
};

}  // namespace visualization
}  // namespace quickviz

#endif  // VISUALIZATION_SELECTION_POINT_CLOUD_SELECTOR_HPP