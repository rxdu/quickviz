/*
 * @file pcl_visualization.hpp
 * @date Dec 2024
 * @brief High-level utilities for visualizing PCL algorithm results
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_PCL_VISUALIZATION_HPP
#define QUICKVIZ_PCL_VISUALIZATION_HPP

#include <vector>
#include <string>
#include <glm/glm.hpp>

// Forward declarations
namespace pcl {
struct PointIndices;
template<typename PointT>
class PointCloud;
}

namespace quickviz {
class PointCloud;
}

namespace quickviz {
namespace pcl_bridge {

/**
 * @brief Visualization utilities for PCL algorithm results
 */
namespace visualization {

/**
 * @brief Color generator for clusters - creates distinct colors for visualization
 * @param num_clusters Number of clusters to generate colors for
 * @return Vector of RGB colors as glm::vec3
 */
std::vector<glm::vec3> GenerateClusterColors(size_t num_clusters);

/**
 * @brief Visualize PCL clustering results by highlighting different clusters
 * @param renderer_cloud The renderer point cloud to modify
 * @param cluster_indices Vector of PCL PointIndices representing clusters
 * @param cluster_colors Optional custom colors for each cluster
 * @return True if visualization was successfully applied
 */
bool VisualizePCLClusters(PointCloud& renderer_cloud,
                         const std::vector<pcl::PointIndices>& cluster_indices,
                         const std::vector<glm::vec3>& cluster_colors = {});

/**
 * @brief Highlight selected points in the renderer
 * @param renderer_cloud The renderer point cloud to modify
 * @param selected_indices Indices of points to highlight
 * @param highlight_color Color to use for highlighting
 * @param highlight_size Optional size multiplier for highlighted points
 * @return True if highlighting was successfully applied
 */
bool HighlightSelectedPoints(PointCloud& renderer_cloud,
                           const std::vector<size_t>& selected_indices,
                           const glm::vec3& highlight_color,
                           float highlight_size = 1.5f);

/**
 * @brief Visualize plane/surface segmentation results
 * @param renderer_cloud The renderer point cloud to modify
 * @param plane_indices Indices of points belonging to detected planes
 * @param plane_colors Colors for each plane
 * @param background_color Color for non-plane points
 * @return True if visualization was successfully applied
 */
bool VisualizePlaneSegmentation(PointCloud& renderer_cloud,
                               const std::vector<std::vector<size_t>>& plane_indices,
                               const std::vector<glm::vec3>& plane_colors,
                               const glm::vec3& background_color = glm::vec3(0.3f, 0.3f, 0.3f));

/**
 * @brief Create bounding box visualization data
 * @param min_corner Minimum corner of the bounding box
 * @param max_corner Maximum corner of the bounding box
 * @param color Color of the bounding box
 * @return Vector of line segments representing the bounding box edges
 */
std::vector<std::pair<glm::vec3, glm::vec3>> CreateBoundingBoxLines(
    const glm::vec3& min_corner,
    const glm::vec3& max_corner,
    const glm::vec3& color = glm::vec3(1.0f, 0.0f, 0.0f));

/**
 * @brief Generate statistics text for algorithm results
 * @param cluster_indices Clustering results
 * @return Formatted string with cluster statistics
 */
std::string GenerateClusterStatistics(const std::vector<pcl::PointIndices>& cluster_indices);

/**
 * @brief Calculate centroid of point cluster
 * @tparam PCLPointT PCL point type
 * @param pcl_cloud Source point cloud
 * @param cluster_indices Indices of points in the cluster
 * @return Centroid position as glm::vec3
 */
template<typename PCLPointT>
glm::vec3 CalculateClusterCentroid(const pcl::PointCloud<PCLPointT>& pcl_cloud,
                                  const pcl::PointIndices& cluster_indices);

/**
 * @brief Create normal vector visualization data
 * @tparam PCLPointT PCL point type
 * @param pcl_cloud Source point cloud
 * @param normals Point normal vectors (as glm::vec3)
 * @param indices Indices of points to show normals for (empty = all points)
 * @param scale Scale factor for normal vector length
 * @return Vector of line segments representing normal vectors
 */
template<typename PCLPointT>
std::vector<std::pair<glm::vec3, glm::vec3>> CreateNormalVectorLines(
    const pcl::PointCloud<PCLPointT>& pcl_cloud,
    const std::vector<glm::vec3>& normals,
    const std::vector<size_t>& indices = {},
    float scale = 0.1f);

/**
 * @brief Quality assessment for visualization results
 */
struct VisualizationQuality {
    size_t total_points;
    size_t highlighted_points;
    size_t num_colors_used;
    bool has_color_conflicts;
    float coverage_percentage;
};

/**
 * @brief Assess quality of current visualization
 * @param renderer_cloud The renderer point cloud to analyze
 * @return Quality assessment metrics
 */
VisualizationQuality AssessVisualizationQuality(const PointCloud& renderer_cloud);

} // namespace visualization
} // namespace pcl_bridge
} // namespace quickviz

#endif // QUICKVIZ_PCL_VISUALIZATION_HPP