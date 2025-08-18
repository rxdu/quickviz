/*
 * @file pcl_conversions.cpp
 * @date Dec 2024
 * @brief Implementation of PCL bridge utilities
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "renderer/pcl_bridge/pcl_conversions.hpp"
#include "renderer/renderable/point_cloud.hpp"

// Include PCL headers only in implementation
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/memory.h>

namespace quickviz {
namespace pcl_bridge {

template<typename PCLPointT>
void ImportFromPCL(const pcl::PointCloud<PCLPointT>& pcl_cloud,
                   PointConverter<PCLPointT> point_converter,
                   PointCloud& renderer_cloud,
                   PointCloud::ColorMode color_mode) {
    std::vector<glm::vec4> points;
    points.reserve(pcl_cloud.points.size());

    for (const auto& pcl_point : pcl_cloud.points) {
        points.push_back(point_converter(pcl_point));
    }

    renderer_cloud.SetPoints(points, color_mode);
}

template<typename PCLPointT>
void ImportFromPCLWithColors(const pcl::PointCloud<PCLPointT>& pcl_cloud,
                            PointConverter<PCLPointT> point_converter,
                            ColorConverter<PCLPointT> color_converter,
                            PointCloud& renderer_cloud) {
    std::vector<glm::vec3> points_3d;
    std::vector<glm::vec3> colors;
    points_3d.reserve(pcl_cloud.points.size());
    colors.reserve(pcl_cloud.points.size());

    for (const auto& pcl_point : pcl_cloud.points) {
        glm::vec4 point_4d = point_converter(pcl_point);
        points_3d.push_back(glm::vec3(point_4d.x, point_4d.y, point_4d.z));
        colors.push_back(color_converter(pcl_point));
    }

    renderer_cloud.SetPoints(points_3d, colors);
}

template<typename PCLPointT>
typename pcl::PointCloud<PCLPointT>::Ptr ExportToPCL(
    const PointCloud& renderer_cloud,
    const std::vector<size_t>& selected_indices,
    std::function<PCLPointT(const glm::vec4&)> point_converter) {
    
    auto pcl_cloud = pcl::make_shared<pcl::PointCloud<PCLPointT>>();
    
    // Get points from renderer (this would need to be added to PointCloud API)
    // For now, this is a placeholder - actual implementation depends on 
    // exposing point data from PointCloud class
    
    // TODO: Add GetPoints() method to PointCloud class
    // const auto& renderer_points = renderer_cloud.GetPoints();
    
    // pcl_cloud->points.reserve(selected_indices.size());
    // for (size_t idx : selected_indices) {
    //     if (idx < renderer_points.size()) {
    //         pcl_cloud->points.push_back(point_converter(renderer_points[idx]));
    //     }
    // }
    
    // pcl_cloud->width = pcl_cloud->points.size();
    // pcl_cloud->height = 1;
    // pcl_cloud->is_dense = true;
    
    return pcl_cloud;
}

template<typename PCLPointT>
typename pcl::PointCloud<PCLPointT>::Ptr ExportToPCL(
    const PointCloud& renderer_cloud,
    std::function<PCLPointT(const glm::vec4&)> point_converter) {
    
    auto pcl_cloud = pcl::make_shared<pcl::PointCloud<PCLPointT>>();
    
    // TODO: Add GetPoints() method to PointCloud class
    // const auto& renderer_points = renderer_cloud.GetPoints();
    
    // pcl_cloud->points.reserve(renderer_points.size());
    // for (const auto& point : renderer_points) {
    //     pcl_cloud->points.push_back(point_converter(point));
    // }
    
    // pcl_cloud->width = pcl_cloud->points.size();
    // pcl_cloud->height = 1;
    // pcl_cloud->is_dense = true;
    
    return pcl_cloud;
}

// Predefined converters implementation
namespace converters {

glm::vec4 PCLXYZToRenderer(const pcl::PointXYZ& point) {
    return glm::vec4(point.x, point.y, point.z, 0.0f);
}

glm::vec4 PCLXYZIToRenderer(const pcl::PointXYZI& point) {
    return glm::vec4(point.x, point.y, point.z, point.intensity);
}

glm::vec4 PCLXYZRGBToRenderer(const pcl::PointXYZRGB& point) {
    // Use luminance as intensity for the w component
    float intensity = 0.299f * point.r + 0.587f * point.g + 0.114f * point.b;
    return glm::vec4(point.x, point.y, point.z, intensity / 255.0f);
}

glm::vec4 PCLXYZRGBAToRenderer(const pcl::PointXYZRGBA& point) {
    // Use luminance as intensity for the w component
    float intensity = 0.299f * point.r + 0.587f * point.g + 0.114f * point.b;
    return glm::vec4(point.x, point.y, point.z, intensity / 255.0f);
}

glm::vec3 PCLXYZRGBToColor(const pcl::PointXYZRGB& point) {
    return glm::vec3(point.r / 255.0f, point.g / 255.0f, point.b / 255.0f);
}

glm::vec3 PCLXYZRGBAToColor(const pcl::PointXYZRGBA& point) {
    return glm::vec3(point.r / 255.0f, point.g / 255.0f, point.b / 255.0f);
}

pcl::PointXYZ RendererToPCLXYZ(const glm::vec4& point) {
    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    return pcl_point;
}

pcl::PointXYZI RendererToPCLXYZI(const glm::vec4& point) {
    pcl::PointXYZI pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    pcl_point.intensity = point.w;
    return pcl_point;
}

pcl::PointXYZRGB RendererToPCLXYZRGB(const glm::vec4& point, const glm::vec3& color) {
    pcl::PointXYZRGB pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    pcl_point.r = static_cast<uint8_t>(color.r * 255.0f);
    pcl_point.g = static_cast<uint8_t>(color.g * 255.0f);
    pcl_point.b = static_cast<uint8_t>(color.b * 255.0f);
    return pcl_point;
}

} // namespace converters

namespace utils {

bool AutoImportFromPCL(const void* pcl_cloud_ptr, 
                      const std::string& point_type,
                      PointCloud& renderer_cloud) {
    if (point_type == "PointXYZ") {
        const auto* cloud = static_cast<const pcl::PointCloud<pcl::PointXYZ>*>(pcl_cloud_ptr);
        ImportFromPCL(*cloud, PointConverter<pcl::PointXYZ>(converters::PCLXYZToRenderer), 
                     renderer_cloud, PointCloud::ColorMode::kHeightField);
        return true;
    } else if (point_type == "PointXYZI") {
        const auto* cloud = static_cast<const pcl::PointCloud<pcl::PointXYZI>*>(pcl_cloud_ptr);
        ImportFromPCL(*cloud, PointConverter<pcl::PointXYZI>(converters::PCLXYZIToRenderer), 
                     renderer_cloud, PointCloud::ColorMode::kScalarField);
        return true;
    } else if (point_type == "PointXYZRGB") {
        const auto* cloud = static_cast<const pcl::PointCloud<pcl::PointXYZRGB>*>(pcl_cloud_ptr);
        ImportFromPCLWithColors(*cloud, PointConverter<pcl::PointXYZRGB>(converters::PCLXYZRGBToRenderer), 
                               ColorConverter<pcl::PointXYZRGB>(converters::PCLXYZRGBToColor), renderer_cloud);
        return true;
    } else if (point_type == "PointXYZRGBA") {
        const auto* cloud = static_cast<const pcl::PointCloud<pcl::PointXYZRGBA>*>(pcl_cloud_ptr);
        ImportFromPCLWithColors(*cloud, PointConverter<pcl::PointXYZRGBA>(converters::PCLXYZRGBAToRenderer),
                               ColorConverter<pcl::PointXYZRGBA>(converters::PCLXYZRGBAToColor), renderer_cloud);
        return true;
    }
    
    return false; // Unsupported point type
}

PointCloud::ColorMode GetColorModeForPCLType(const std::string& point_type) {
    if (point_type == "PointXYZ") {
        return PointCloud::ColorMode::kHeightField;
    } else if (point_type == "PointXYZI") {
        return PointCloud::ColorMode::kScalarField;
    } else if (point_type == "PointXYZRGB" || point_type == "PointXYZRGBA") {
        return PointCloud::ColorMode::kRGB;
    }
    
    return PointCloud::ColorMode::kHeightField; // Default
}

template<typename PCLPointT>
std::pair<glm::vec3, glm::vec3> CalculateBoundingBox(
    const pcl::PointCloud<PCLPointT>& pcl_cloud) {
    if (pcl_cloud.points.empty()) {
        return {glm::vec3(0.0f), glm::vec3(0.0f)};
    }

    glm::vec3 min_pt(std::numeric_limits<float>::max());
    glm::vec3 max_pt(std::numeric_limits<float>::lowest());

    for (const auto& point : pcl_cloud.points) {
        min_pt.x = std::min(min_pt.x, point.x);
        min_pt.y = std::min(min_pt.y, point.y);
        min_pt.z = std::min(min_pt.z, point.z);
        
        max_pt.x = std::max(max_pt.x, point.x);
        max_pt.y = std::max(max_pt.y, point.y);
        max_pt.z = std::max(max_pt.z, point.z);
    }

    return {min_pt, max_pt};
}

// Explicit template instantiations for common PCL types
template std::pair<glm::vec3, glm::vec3> CalculateBoundingBox<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>&);
template std::pair<glm::vec3, glm::vec3> CalculateBoundingBox<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>&);
template std::pair<glm::vec3, glm::vec3> CalculateBoundingBox<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>&);
template std::pair<glm::vec3, glm::vec3> CalculateBoundingBox<pcl::PointXYZRGBA>(
    const pcl::PointCloud<pcl::PointXYZRGBA>&);

} // namespace utils

// Explicit template instantiations for common use cases
template void ImportFromPCL<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>&,
                                           PointConverter<pcl::PointXYZ>, PointCloud&, PointCloud::ColorMode);
template void ImportFromPCL<pcl::PointXYZI>(const pcl::PointCloud<pcl::PointXYZI>&,
                                            PointConverter<pcl::PointXYZI>, PointCloud&, PointCloud::ColorMode);
template void ImportFromPCL<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB>&,
                                              PointConverter<pcl::PointXYZRGB>, PointCloud&, PointCloud::ColorMode);

template void ImportFromPCLWithColors<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB>&,
                                                        PointConverter<pcl::PointXYZRGB>,
                                                        ColorConverter<pcl::PointXYZRGB>, PointCloud&);
template void ImportFromPCLWithColors<pcl::PointXYZRGBA>(const pcl::PointCloud<pcl::PointXYZRGBA>&,
                                                         PointConverter<pcl::PointXYZRGBA>,
                                                         ColorConverter<pcl::PointXYZRGBA>, PointCloud&);

} // namespace pcl_bridge
} // namespace quickviz