/*
 * @file pcl_conversions.hpp
 * @date Dec 2024
 * @brief Template-based utilities for converting between PCL and QuickViz renderer types
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_PCL_CONVERSIONS_HPP
#define QUICKVIZ_PCL_CONVERSIONS_HPP

#include <vector>
#include <functional>
#include <string>
#include <glm/glm.hpp>
#include "gldraw/renderable/point_cloud.hpp"

// Forward declarations for PCL types to avoid requiring PCL headers in this file
namespace pcl {
template<typename PointT>
class PointCloud;

struct PointXYZ;
struct PointXYZI;
struct PointXYZRGB;
struct PointXYZRGBA;
}

namespace quickviz {
namespace pcl_bridge {

/**
 * @brief Generic converter function type for PCL point to renderer point conversion
 * @tparam PCLPointT PCL point type (e.g., pcl::PointXYZ, pcl::PointXYZRGB)
 */
template<typename PCLPointT>
using PointConverter = std::function<glm::vec4(const PCLPointT&)>;

/**
 * @brief Generic converter function type for PCL point to RGB color conversion
 * @tparam PCLPointT PCL point type with color information
 */
template<typename PCLPointT>
using ColorConverter = std::function<glm::vec3(const PCLPointT&)>;

/**
 * @brief Import PCL point cloud to QuickViz renderer format
 * @tparam PCLPointT PCL point type
 * @param pcl_cloud Input PCL point cloud
 * @param point_converter Function to convert PCL point to glm::vec4 (x,y,z,intensity)
 * @param renderer_cloud Output renderer point cloud
 * @param color_mode Color mode for the renderer point cloud
 */
template<typename PCLPointT>
void ImportFromPCL(const pcl::PointCloud<PCLPointT>& pcl_cloud,
                   PointConverter<PCLPointT> point_converter,
                   PointCloud& renderer_cloud,
                   PointCloud::ColorMode color_mode);

/**
 * @brief Import PCL point cloud with RGB colors to QuickViz renderer
 * @tparam PCLPointT PCL point type with color information
 * @param pcl_cloud Input PCL point cloud
 * @param point_converter Function to convert PCL point to glm::vec4
 * @param color_converter Function to extract RGB color from PCL point
 * @param renderer_cloud Output renderer point cloud
 */
template<typename PCLPointT>
void ImportFromPCLWithColors(const pcl::PointCloud<PCLPointT>& pcl_cloud,
                            PointConverter<PCLPointT> point_converter,
                            ColorConverter<PCLPointT> color_converter,
                            PointCloud& renderer_cloud);

/**
 * @brief Export selected points from renderer to PCL format
 * @tparam PCLPointT Target PCL point type
 * @param renderer_cloud Input renderer point cloud
 * @param selected_indices Indices of selected points
 * @param point_converter Function to convert renderer point to PCL point
 * @return PCL point cloud containing only selected points
 */
template<typename PCLPointT>
typename pcl::PointCloud<PCLPointT>::Ptr ExportToPCL(
    const PointCloud& renderer_cloud,
    const std::vector<size_t>& selected_indices,
    std::function<PCLPointT(const glm::vec4&)> point_converter);

/**
 * @brief Export all points from renderer to PCL format
 * @tparam PCLPointT Target PCL point type
 * @param renderer_cloud Input renderer point cloud
 * @param point_converter Function to convert renderer point to PCL point
 * @return PCL point cloud containing all points
 */
template<typename PCLPointT>
typename pcl::PointCloud<PCLPointT>::Ptr ExportToPCL(
    const PointCloud& renderer_cloud,
    std::function<PCLPointT(const glm::vec4&)> point_converter);

// Predefined converters for common PCL types
namespace converters {

/**
 * @brief Standard converter for PCL PointXYZ to renderer format
 */
glm::vec4 PCLXYZToRenderer(const pcl::PointXYZ& point);

/**
 * @brief Standard converter for PCL PointXYZI to renderer format
 */
glm::vec4 PCLXYZIToRenderer(const pcl::PointXYZI& point);

/**
 * @brief Standard converter for PCL PointXYZRGB to renderer format
 */
glm::vec4 PCLXYZRGBToRenderer(const pcl::PointXYZRGB& point);

/**
 * @brief Standard converter for PCL PointXYZRGBA to renderer format
 */
glm::vec4 PCLXYZRGBAToRenderer(const pcl::PointXYZRGBA& point);

/**
 * @brief RGB color extractor for PCL PointXYZRGB
 */
glm::vec3 PCLXYZRGBToColor(const pcl::PointXYZRGB& point);

/**
 * @brief RGB color extractor for PCL PointXYZRGBA
 */
glm::vec3 PCLXYZRGBAToColor(const pcl::PointXYZRGBA& point);

/**
 * @brief Standard converter from renderer to PCL PointXYZ
 */
pcl::PointXYZ RendererToPCLXYZ(const glm::vec4& point);

/**
 * @brief Standard converter from renderer to PCL PointXYZI
 */
pcl::PointXYZI RendererToPCLXYZI(const glm::vec4& point);

/**
 * @brief Standard converter from renderer to PCL PointXYZRGB (requires color)
 */
pcl::PointXYZRGB RendererToPCLXYZRGB(const glm::vec4& point, const glm::vec3& color);

} // namespace converters

/**
 * @brief Utility functions for common conversion workflows
 */
namespace utils {

/**
 * @brief Detect PCL point cloud type and import automatically
 * @param pcl_cloud_ptr Polymorphic pointer to PCL point cloud
 * @param renderer_cloud Output renderer point cloud
 * @return True if conversion was successful
 */
bool AutoImportFromPCL(const void* pcl_cloud_ptr, 
                      const std::string& point_type,
                      PointCloud& renderer_cloud);

/**
 * @brief Get appropriate color mode based on PCL point type
 * @param point_type String identifier for PCL point type
 * @return Recommended color mode for renderer
 */
PointCloud::ColorMode GetColorModeForPCLType(const std::string& point_type);

/**
 * @brief Calculate bounding box for PCL point cloud
 * @tparam PCLPointT PCL point type
 * @param pcl_cloud Input PCL point cloud
 * @return Pair of min and max corners as glm::vec3
 */
template<typename PCLPointT>
std::pair<glm::vec3, glm::vec3> CalculateBoundingBox(
    const pcl::PointCloud<PCLPointT>& pcl_cloud);

} // namespace utils

} // namespace pcl_bridge
} // namespace quickviz

#endif // QUICKVIZ_PCL_CONVERSIONS_HPP