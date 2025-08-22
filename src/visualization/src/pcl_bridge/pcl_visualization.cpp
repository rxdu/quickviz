/*
 * @file pcl_visualization.cpp
 * @date Dec 2024
 * @brief Implementation of PCL visualization utilities
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "visualization/pcl_bridge/pcl_visualization.hpp"
#include "gldraw/renderable/point_cloud.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <numeric>

namespace quickviz {
namespace pcl_bridge {
namespace visualization {

std::vector<glm::vec3> GenerateClusterColors(size_t num_clusters) {
    std::vector<glm::vec3> colors;
    colors.reserve(num_clusters);
    
    // Use HSV color space to generate distinct colors
    for (size_t i = 0; i < num_clusters; ++i) {
        float hue = (360.0f * i) / num_clusters;
        float saturation = 0.8f;
        float value = 0.9f;
        
        // Convert HSV to RGB
        float c = value * saturation;
        float x = c * (1.0f - std::abs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
        float m = value - c;
        
        glm::vec3 rgb;
        if (hue < 60.0f) {
            rgb = glm::vec3(c, x, 0.0f);
        } else if (hue < 120.0f) {
            rgb = glm::vec3(x, c, 0.0f);
        } else if (hue < 180.0f) {
            rgb = glm::vec3(0.0f, c, x);
        } else if (hue < 240.0f) {
            rgb = glm::vec3(0.0f, x, c);
        } else if (hue < 300.0f) {
            rgb = glm::vec3(x, 0.0f, c);
        } else {
            rgb = glm::vec3(c, 0.0f, x);
        }
        
        colors.push_back(rgb + glm::vec3(m));
    }
    
    return colors;
}

bool VisualizePCLClusters(PointCloud& renderer_cloud,
                         const std::vector<pcl::PointIndices>& cluster_indices,
                         const std::vector<glm::vec3>& cluster_colors) {
    // Generate colors if not provided
    std::vector<glm::vec3> colors = cluster_colors;
    if (colors.empty()) {
        colors = GenerateClusterColors(cluster_indices.size());
    } else if (colors.size() < cluster_indices.size()) {
        // Extend colors if not enough provided
        auto additional_colors = GenerateClusterColors(cluster_indices.size() - colors.size());
        colors.insert(colors.end(), additional_colors.begin(), additional_colors.end());
    }
    
    // TODO: This requires extending the PointCloud API to support per-point coloring
    // For now, this is a placeholder implementation
    
    // Proposed API addition to PointCloud class:
    // renderer_cloud.SetPointColors(point_index_to_color_map);
    // or
    // renderer_cloud.HighlightPoints(indices, color);
    
    // The actual implementation would:
    // 1. Get current point data from renderer_cloud
    // 2. Create a color map for all points
    // 3. For each cluster, assign the cluster color to its points
    // 4. Apply the color map to the renderer_cloud
    
    return true; // Placeholder return
}

bool HighlightSelectedPoints(PointCloud& renderer_cloud,
                           const std::vector<size_t>& selected_indices,
                           const glm::vec3& highlight_color,
                           float highlight_size) {
    // TODO: This requires extending the PointCloud API
    // Proposed API:
    // renderer_cloud.HighlightPoints(selected_indices, highlight_color, highlight_size);
    
    return true; // Placeholder return
}

bool VisualizePlaneSegmentation(PointCloud& renderer_cloud,
                               const std::vector<std::vector<size_t>>& plane_indices,
                               const std::vector<glm::vec3>& plane_colors,
                               const glm::vec3& background_color) {
    // TODO: Similar to cluster visualization, this needs enhanced PointCloud API
    // Would set background color for all points, then override with plane colors
    
    return true; // Placeholder return
}

std::vector<std::pair<glm::vec3, glm::vec3>> CreateBoundingBoxLines(
    const glm::vec3& min_corner,
    const glm::vec3& max_corner,
    const glm::vec3& color) {
    
    std::vector<std::pair<glm::vec3, glm::vec3>> lines;
    lines.reserve(12); // A cube has 12 edges
    
    // Define 8 corners of the bounding box
    glm::vec3 corners[8] = {
        glm::vec3(min_corner.x, min_corner.y, min_corner.z), // 0: min
        glm::vec3(max_corner.x, min_corner.y, min_corner.z), // 1
        glm::vec3(max_corner.x, max_corner.y, min_corner.z), // 2
        glm::vec3(min_corner.x, max_corner.y, min_corner.z), // 3
        glm::vec3(min_corner.x, min_corner.y, max_corner.z), // 4
        glm::vec3(max_corner.x, min_corner.y, max_corner.z), // 5
        glm::vec3(max_corner.x, max_corner.y, max_corner.z), // 6: max
        glm::vec3(min_corner.x, max_corner.y, max_corner.z)  // 7
    };
    
    // Bottom face (z = min)
    lines.push_back({corners[0], corners[1]});
    lines.push_back({corners[1], corners[2]});
    lines.push_back({corners[2], corners[3]});
    lines.push_back({corners[3], corners[0]});
    
    // Top face (z = max)
    lines.push_back({corners[4], corners[5]});
    lines.push_back({corners[5], corners[6]});
    lines.push_back({corners[6], corners[7]});
    lines.push_back({corners[7], corners[4]});
    
    // Vertical edges
    lines.push_back({corners[0], corners[4]});
    lines.push_back({corners[1], corners[5]});
    lines.push_back({corners[2], corners[6]});
    lines.push_back({corners[3], corners[7]});
    
    return lines;
}

std::string GenerateClusterStatistics(const std::vector<pcl::PointIndices>& cluster_indices) {
    std::ostringstream ss;
    ss << "Clustering Results:\n";
    ss << "Number of clusters: " << cluster_indices.size() << "\n";
    
    if (!cluster_indices.empty()) {
        size_t total_points = 0;
        size_t min_size = std::numeric_limits<size_t>::max();
        size_t max_size = 0;
        
        for (size_t i = 0; i < cluster_indices.size(); ++i) {
            size_t cluster_size = cluster_indices[i].indices.size();
            total_points += cluster_size;
            min_size = std::min(min_size, cluster_size);
            max_size = std::max(max_size, cluster_size);
            
            ss << "  Cluster " << i << ": " << cluster_size << " points\n";
        }
        
        ss << "Total clustered points: " << total_points << "\n";
        ss << "Cluster size range: " << min_size << " - " << max_size << " points\n";
        ss << "Average cluster size: " << (total_points / cluster_indices.size()) << " points\n";
    }
    
    return ss.str();
}

template<typename PCLPointT>
glm::vec3 CalculateClusterCentroid(const pcl::PointCloud<PCLPointT>& pcl_cloud,
                                  const pcl::PointIndices& cluster_indices) {
    if (cluster_indices.indices.empty()) {
        return glm::vec3(0.0f);
    }
    
    glm::vec3 centroid(0.0f);
    for (const auto& idx : cluster_indices.indices) {
        if (idx < pcl_cloud.points.size()) {
            const auto& point = pcl_cloud.points[idx];
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
    }
    
    float num_points = static_cast<float>(cluster_indices.indices.size());
    return centroid / num_points;
}

template<typename PCLPointT>
std::vector<std::pair<glm::vec3, glm::vec3>> CreateNormalVectorLines(
    const pcl::PointCloud<PCLPointT>& pcl_cloud,
    const std::vector<glm::vec3>& normals,
    const std::vector<size_t>& indices,
    float scale) {
    
    std::vector<std::pair<glm::vec3, glm::vec3>> lines;
    
    // Use provided indices or all points if none specified
    std::vector<size_t> point_indices = indices;
    if (point_indices.empty()) {
        point_indices.resize(std::min(pcl_cloud.points.size(), normals.size()));
        std::iota(point_indices.begin(), point_indices.end(), 0);
    }
    
    lines.reserve(point_indices.size());
    
    for (size_t idx : point_indices) {
        if (idx < pcl_cloud.points.size() && idx < normals.size()) {
            const auto& point = pcl_cloud.points[idx];
            glm::vec3 start(point.x, point.y, point.z);
            glm::vec3 end = start + normals[idx] * scale;
            lines.push_back({start, end});
        }
    }
    
    return lines;
}

VisualizationQuality AssessVisualizationQuality(const PointCloud& renderer_cloud) {
    VisualizationQuality quality;
    
    // TODO: This requires access to internal PointCloud data
    // Placeholder implementation
    quality.total_points = 0;
    quality.highlighted_points = 0;
    quality.num_colors_used = 1;
    quality.has_color_conflicts = false;
    quality.coverage_percentage = 100.0f;
    
    return quality;
}

// Explicit template instantiations for common PCL types
template glm::vec3 CalculateClusterCentroid<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud,
    const pcl::PointIndices& cluster_indices);

template glm::vec3 CalculateClusterCentroid<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud,
    const pcl::PointIndices& cluster_indices);

template glm::vec3 CalculateClusterCentroid<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud,
    const pcl::PointIndices& cluster_indices);

template std::vector<std::pair<glm::vec3, glm::vec3>> CreateNormalVectorLines<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud,
    const std::vector<glm::vec3>& normals,
    const std::vector<size_t>& indices,
    float scale);

template std::vector<std::pair<glm::vec3, glm::vec3>> CreateNormalVectorLines<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>& pcl_cloud,
    const std::vector<glm::vec3>& normals,
    const std::vector<size_t>& indices,
    float scale);

template std::vector<std::pair<glm::vec3, glm::vec3>> CreateNormalVectorLines<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud,
    const std::vector<glm::vec3>& normals,
    const std::vector<size_t>& indices,
    float scale);

} // namespace visualization
} // namespace pcl_bridge
} // namespace quickviz