/*
 * @file pcl_loader.cpp
 * @date Dec 2024
 * @brief Implementation of point cloud file loader with automatic field detection
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "visualization/pcl_bridge/pcl_loader.hpp"
#include "visualization/pcl_bridge/pcl_conversions.hpp"
#include "gldraw/renderable/point_cloud.hpp"

#include <algorithm>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <cctype>
#include <limits>

// Include PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/memory.h>

namespace quickviz {
namespace pcl_bridge {

// PointCloudMetadata implementation
std::string PointCloudMetadata::GetRecommendedPCLType() const {
  if (fields.HasRGBAColor()) {
    return "PointXYZRGBA";
  } else if (fields.HasRGBColor()) {
    return "PointXYZRGB";
  } else if (fields.has_intensity) {
    return "PointXYZI";
  } else if (fields.HasXYZ()) {
    return "PointXYZ";
  }
  return "PointXYZ"; // Default fallback
}

// PointCloudLoader implementation
PointCloudMetadata PointCloudLoader::Load(const std::string& filename,
                                         PointCloud& renderer_cloud,
                                         Format format,
                                         ProgressCallback progress_callback) {
  ValidateFileAccess(filename);
  
  Format actual_format = (format == Format::kAutoDetect) ? DetectFormat(filename) : format;
  
  switch (actual_format) {
    case Format::kPCD:
      return LoadPCD(filename, renderer_cloud, progress_callback);
    case Format::kPLY:
      return LoadPLY(filename, renderer_cloud, progress_callback);
    default:
      throw UnsupportedFormatException(FormatToString(actual_format));
  }
}

PointCloudMetadata PointCloudLoader::LoadPCD(const std::string& filename,
                                            PointCloud& renderer_cloud,
                                            ProgressCallback progress_callback) {
  if (progress_callback) {
    progress_callback(0.1f, "Analyzing PCD file fields...");
  }
  
  PointCloudFields fields = DetectPCDFields(filename);
  
  if (progress_callback) {
    progress_callback(0.2f, "Loading point cloud data...");
  }
  
  return LoadPCDWithAutoType(filename, renderer_cloud, fields, progress_callback);
}

PointCloudMetadata PointCloudLoader::LoadPLY(const std::string& filename,
                                            PointCloud& renderer_cloud,
                                            ProgressCallback progress_callback) {
  if (progress_callback) {
    progress_callback(0.1f, "Analyzing PLY file fields...");
  }
  
  PointCloudFields fields = DetectPLYFields(filename);
  
  if (progress_callback) {
    progress_callback(0.2f, "Loading point cloud data...");
  }
  
  return LoadPLYWithAutoType(filename, renderer_cloud, fields, progress_callback);
}

PointCloudMetadata PointCloudLoader::AnalyzeFields(const std::string& filename,
                                                  Format format) {
  ValidateFileAccess(filename);
  
  Format actual_format = (format == Format::kAutoDetect) ? DetectFormat(filename) : format;
  
  PointCloudMetadata metadata;
  metadata.filename = filename;
  metadata.format = FormatToString(actual_format);
  metadata.file_size_mb = GetFileSizeMB(filename);
  
  switch (actual_format) {
    case Format::kPCD:
      metadata.fields = DetectPCDFields(filename);
      break;
    case Format::kPLY:
      metadata.fields = DetectPLYFields(filename);
      break;
    default:
      throw UnsupportedFormatException(FormatToString(actual_format));
  }
  
  metadata.detected_pcl_type = metadata.GetRecommendedPCLType();
  return metadata;
}

PointCloudLoader::Format PointCloudLoader::DetectFormat(const std::string& filename) {
  std::filesystem::path path(filename);
  std::string extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
  
  if (extension == ".pcd") {
    return Format::kPCD;
  } else if (extension == ".ply") {
    return Format::kPLY;
  }
  
  throw UnsupportedFormatException("Unknown file extension: " + extension);
}

bool PointCloudLoader::IsFormatSupported(Format format) {
  return format == Format::kPCD || format == Format::kPLY || format == Format::kAutoDetect;
}

std::vector<std::string> PointCloudLoader::GetSupportedExtensions() {
  return {".pcd", ".ply"};
}

// Private methods implementation
PointCloudFields PointCloudLoader::DetectPCDFields(const std::string& filename) {
  return field_detector::ParsePCDHeader(filename);
}

PointCloudFields PointCloudLoader::DetectPLYFields(const std::string& filename) {
  return field_detector::ParsePLYHeader(filename);
}

PointCloudMetadata PointCloudLoader::LoadPCDWithAutoType(
    const std::string& filename,
    PointCloud& renderer_cloud,
    const PointCloudFields& fields,
    ProgressCallback progress_callback) {
  
  std::string optimal_type = DetermineOptimalPCLType(fields);
  
  if (progress_callback) {
    progress_callback(0.3f, "Loading as " + optimal_type + "...");
  }
  
  if (optimal_type == "PointXYZRGBA") {
    auto [cloud, metadata] = LoadPCDInternal<pcl::PointXYZRGBA>(filename, progress_callback);
    if (progress_callback) progress_callback(0.8f, "Converting to renderer format...");
    utils::AutoImportFromPCL(cloud.get(), "PointXYZRGBA", renderer_cloud);
    if (progress_callback) progress_callback(1.0f, "Loading complete");
    return metadata;
  } else if (optimal_type == "PointXYZRGB") {
    auto [cloud, metadata] = LoadPCDInternal<pcl::PointXYZRGB>(filename, progress_callback);
    if (progress_callback) progress_callback(0.8f, "Converting to renderer format...");
    utils::AutoImportFromPCL(cloud.get(), "PointXYZRGB", renderer_cloud);
    if (progress_callback) progress_callback(1.0f, "Loading complete");
    return metadata;
  } else if (optimal_type == "PointXYZI") {
    auto [cloud, metadata] = LoadPCDInternal<pcl::PointXYZI>(filename, progress_callback);
    if (progress_callback) progress_callback(0.8f, "Converting to renderer format...");
    utils::AutoImportFromPCL(cloud.get(), "PointXYZI", renderer_cloud);
    if (progress_callback) progress_callback(1.0f, "Loading complete");
    return metadata;
  } else {
    auto [cloud, metadata] = LoadPCDInternal<pcl::PointXYZ>(filename, progress_callback);
    if (progress_callback) progress_callback(0.8f, "Converting to renderer format...");
    utils::AutoImportFromPCL(cloud.get(), "PointXYZ", renderer_cloud);
    if (progress_callback) progress_callback(1.0f, "Loading complete");
    return metadata;
  }
}

PointCloudMetadata PointCloudLoader::LoadPLYWithAutoType(
    const std::string& filename,
    PointCloud& renderer_cloud,
    const PointCloudFields& fields,
    ProgressCallback progress_callback) {
  
  std::string optimal_type = DetermineOptimalPCLType(fields);
  
  if (progress_callback) {
    progress_callback(0.3f, "Loading as " + optimal_type + "...");
  }
  
  if (optimal_type == "PointXYZRGBA") {
    auto [cloud, metadata] = LoadPLYInternal<pcl::PointXYZRGBA>(filename, progress_callback);
    if (progress_callback) progress_callback(0.8f, "Converting to renderer format...");
    utils::AutoImportFromPCL(cloud.get(), "PointXYZRGBA", renderer_cloud);
    if (progress_callback) progress_callback(1.0f, "Loading complete");
    return metadata;
  } else if (optimal_type == "PointXYZRGB") {
    auto [cloud, metadata] = LoadPLYInternal<pcl::PointXYZRGB>(filename, progress_callback);
    if (progress_callback) progress_callback(0.8f, "Converting to renderer format...");
    utils::AutoImportFromPCL(cloud.get(), "PointXYZRGB", renderer_cloud);
    if (progress_callback) progress_callback(1.0f, "Loading complete");
    return metadata;
  } else if (optimal_type == "PointXYZI") {
    auto [cloud, metadata] = LoadPLYInternal<pcl::PointXYZI>(filename, progress_callback);
    if (progress_callback) progress_callback(0.8f, "Converting to renderer format...");
    utils::AutoImportFromPCL(cloud.get(), "PointXYZI", renderer_cloud);
    if (progress_callback) progress_callback(1.0f, "Loading complete");
    return metadata;
  } else {
    auto [cloud, metadata] = LoadPLYInternal<pcl::PointXYZ>(filename, progress_callback);
    if (progress_callback) progress_callback(0.8f, "Converting to renderer format...");
    utils::AutoImportFromPCL(cloud.get(), "PointXYZ", renderer_cloud);
    if (progress_callback) progress_callback(1.0f, "Loading complete");
    return metadata;
  }
}

template<typename PCLPointT>
std::pair<typename pcl::PointCloud<PCLPointT>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadPCDInternal(const std::string& filename, ProgressCallback progress_callback) {
  auto cloud = pcl::make_shared<pcl::PointCloud<PCLPointT>>();
  
  if (pcl::io::loadPCDFile<PCLPointT>(filename, *cloud) == -1) {
    throw CorruptedFileException("Failed to load PCD file: " + filename);
  }
  
  if (progress_callback) {
    progress_callback(0.7f, "Calculating metadata...");
  }
  
  PointCloudFields fields = DetectPCDFields(filename);
  PointCloudMetadata metadata = CalculateMetadata(filename, "PCD", *cloud, fields);
  
  return {cloud, metadata};
}

template<typename PCLPointT>
std::pair<typename pcl::PointCloud<PCLPointT>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadPLYInternal(const std::string& filename, ProgressCallback progress_callback) {
  auto cloud = pcl::make_shared<pcl::PointCloud<PCLPointT>>();
  
  if (pcl::io::loadPLYFile<PCLPointT>(filename, *cloud) == -1) {
    throw CorruptedFileException("Failed to load PLY file: " + filename);
  }
  
  if (progress_callback) {
    progress_callback(0.7f, "Calculating metadata...");
  }
  
  PointCloudFields fields = DetectPLYFields(filename);
  PointCloudMetadata metadata = CalculateMetadata(filename, "PLY", *cloud, fields);
  
  return {cloud, metadata};
}

template<typename PCLPointT>
PointCloudMetadata PointCloudLoader::CalculateMetadata(
    const std::string& filename,
    const std::string& format,
    const pcl::PointCloud<PCLPointT>& cloud,
    const PointCloudFields& fields) {
  
  PointCloudMetadata metadata;
  metadata.filename = filename;
  metadata.format = format;
  metadata.detected_pcl_type = GetPointTypeString<PCLPointT>();
  metadata.point_count = cloud.points.size();
  metadata.fields = fields;
  metadata.file_size_mb = GetFileSizeMB(filename);
  
  // Calculate bounding box
  if (!cloud.points.empty()) {
    auto [min_pt, max_pt] = utils::CalculateBoundingBox(cloud);
    metadata.min_bounds = min_pt;
    metadata.max_bounds = max_pt;
  }
  
  return metadata;
}

void PointCloudLoader::ValidateFileAccess(const std::string& filename) {
  if (!std::filesystem::exists(filename)) {
    throw FileNotFoundException(filename);
  }
  
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw PointCloudLoaderException("Cannot open file: " + filename);
  }
}

double PointCloudLoader::GetFileSizeMB(const std::string& filename) {
  try {
    auto file_size = std::filesystem::file_size(filename);
    return static_cast<double>(file_size) / (1024.0 * 1024.0);
  } catch (const std::filesystem::filesystem_error&) {
    return 0.0;
  }
}

std::string PointCloudLoader::FormatToString(Format format) {
  switch (format) {
    case Format::kPCD: return "PCD";
    case Format::kPLY: return "PLY";
    case Format::kAutoDetect: return "AUTO_DETECT";
    default: return "UNKNOWN";
  }
}

template<typename PCLPointT>
std::string PointCloudLoader::GetPointTypeString() {
  if (std::is_same_v<PCLPointT, pcl::PointXYZ>) {
    return "PointXYZ";
  } else if (std::is_same_v<PCLPointT, pcl::PointXYZI>) {
    return "PointXYZI";
  } else if (std::is_same_v<PCLPointT, pcl::PointXYZRGB>) {
    return "PointXYZRGB";
  } else if (std::is_same_v<PCLPointT, pcl::PointXYZRGBA>) {
    return "PointXYZRGBA";
  }
  return "Unknown";
}

std::string PointCloudLoader::DetermineOptimalPCLType(const PointCloudFields& fields) {
  if (fields.HasRGBAColor()) {
    return "PointXYZRGBA";
  } else if (fields.HasRGBColor()) {
    return "PointXYZRGB";
  } else if (fields.has_intensity) {
    return "PointXYZI";
  }
  return "PointXYZ";
}

template<typename PCLPointT>
std::pair<typename pcl::PointCloud<PCLPointT>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadToPCL(const std::string& filename,
                           Format format,
                           ProgressCallback progress_callback) {
  ValidateFileAccess(filename);
  
  Format actual_format = (format == Format::kAutoDetect) ? DetectFormat(filename) : format;
  
  switch (actual_format) {
    case Format::kPCD:
      return LoadPCDInternal<PCLPointT>(filename, progress_callback);
    case Format::kPLY:
      return LoadPLYInternal<PCLPointT>(filename, progress_callback);
    default:
      throw UnsupportedFormatException(FormatToString(actual_format));
  }
}

// Explicit template instantiations for common PCL types
template std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadPCDInternal<pcl::PointXYZ>(const std::string&, ProgressCallback);

template std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadPCDInternal<pcl::PointXYZI>(const std::string&, ProgressCallback);

template std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadPCDInternal<pcl::PointXYZRGB>(const std::string&, ProgressCallback);

template std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadPCDInternal<pcl::PointXYZRGBA>(const std::string&, ProgressCallback);

template std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadPLYInternal<pcl::PointXYZ>(const std::string&, ProgressCallback);

template std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadPLYInternal<pcl::PointXYZI>(const std::string&, ProgressCallback);

template std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadPLYInternal<pcl::PointXYZRGB>(const std::string&, ProgressCallback);

template std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadPLYInternal<pcl::PointXYZRGBA>(const std::string&, ProgressCallback);

// Explicit template instantiations for LoadToPCL
template std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadToPCL<pcl::PointXYZ>(const std::string&, Format, ProgressCallback);

template std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadToPCL<pcl::PointXYZI>(const std::string&, Format, ProgressCallback);

template std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadToPCL<pcl::PointXYZRGB>(const std::string&, Format, ProgressCallback);

template std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, PointCloudMetadata>
PointCloudLoader::LoadToPCL<pcl::PointXYZRGBA>(const std::string&, Format, ProgressCallback);

// field_detector namespace implementation
namespace field_detector {

PointCloudFields ParsePCDHeader(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw FileNotFoundException(filename);
  }
  
  PointCloudFields fields;
  std::string line;
  
  while (std::getline(file, line) && line != "DATA ascii" && line != "DATA binary") {
    if (line.substr(0, 6) == "FIELDS") {
      std::istringstream iss(line);
      std::string token;
      iss >> token; // Skip "FIELDS"
      
      while (iss >> token) {
        std::transform(token.begin(), token.end(), token.begin(), ::tolower);
        
        if (token == "x") fields.has_x = true;
        else if (token == "y") fields.has_y = true;
        else if (token == "z") fields.has_z = true;
        else if (token == "intensity") fields.has_intensity = true;
        else if (token == "rgb") fields.has_rgb = true;
        else if (token == "rgba") fields.has_rgba = true;
        else if (token == "r") fields.has_r = true;
        else if (token == "g") fields.has_g = true;
        else if (token == "b") fields.has_b = true;
        else if (token == "a") fields.has_a = true;
        else if (token == "normal_x") fields.has_normal_x = true;
        else if (token == "normal_y") fields.has_normal_y = true;
        else if (token == "normal_z") fields.has_normal_z = true;
      }
      break;
    }
  }
  
  return fields;
}

PointCloudFields ParsePLYHeader(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw FileNotFoundException(filename);
  }
  
  PointCloudFields fields;
  std::string line;
  
  while (std::getline(file, line) && line != "end_header") {
    if (line.substr(0, 8) == "property") {
      std::istringstream iss(line);
      std::string token, type, name;
      iss >> token >> type >> name; // Skip "property", get type and name
      
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      
      if (name == "x") fields.has_x = true;
      else if (name == "y") fields.has_y = true;
      else if (name == "z") fields.has_z = true;
      else if (name == "intensity") fields.has_intensity = true;
      else if (name == "rgb") fields.has_rgb = true;
      else if (name == "rgba") fields.has_rgba = true;
      else if (name == "red" || name == "r") fields.has_r = true;
      else if (name == "green" || name == "g") fields.has_g = true;
      else if (name == "blue" || name == "b") fields.has_b = true;
      else if (name == "alpha" || name == "a") fields.has_a = true;
      else if (name == "nx" || name == "normal_x") fields.has_normal_x = true;
      else if (name == "ny" || name == "normal_y") fields.has_normal_y = true;
      else if (name == "nz" || name == "normal_z") fields.has_normal_z = true;
    }
  }
  
  return fields;
}

template<typename PCLPointT>
bool IsCompatiblePCLType(const PointCloudFields& fields) {
  if (std::is_same_v<PCLPointT, pcl::PointXYZ>) {
    return fields.HasXYZ();
  } else if (std::is_same_v<PCLPointT, pcl::PointXYZI>) {
    return fields.HasXYZ() && fields.has_intensity;
  } else if (std::is_same_v<PCLPointT, pcl::PointXYZRGB>) {
    return fields.HasXYZ() && fields.HasRGBColor();
  } else if (std::is_same_v<PCLPointT, pcl::PointXYZRGBA>) {
    return fields.HasXYZ() && fields.HasRGBAColor();
  }
  return false;
}

PointCloudFields GetRequiredFields(const std::string& pcl_type) {
  PointCloudFields fields;
  fields.has_x = fields.has_y = fields.has_z = true; // All types need XYZ
  
  if (pcl_type == "PointXYZI") {
    fields.has_intensity = true;
  } else if (pcl_type == "PointXYZRGB") {
    fields.has_r = fields.has_g = fields.has_b = true;
  } else if (pcl_type == "PointXYZRGBA") {
    fields.has_r = fields.has_g = fields.has_b = fields.has_a = true;
  }
  
  return fields;
}

// Explicit template instantiations
template bool IsCompatiblePCLType<pcl::PointXYZ>(const PointCloudFields&);
template bool IsCompatiblePCLType<pcl::PointXYZI>(const PointCloudFields&);
template bool IsCompatiblePCLType<pcl::PointXYZRGB>(const PointCloudFields&);
template bool IsCompatiblePCLType<pcl::PointXYZRGBA>(const PointCloudFields&);

} // namespace field_detector

// Factory implementation
namespace factory {

std::pair<RendererData, PointCloudMetadata> 
RendererFactory::Load(const std::string& filename,
                     PointCloudLoader::Format format,
                     ProgressCallback progress_callback) {
  // Validate file access
  if (!std::filesystem::exists(filename)) {
    throw FileNotFoundException(filename);
  }
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open file: " + filename);
  }
  
  // Auto-detect format if needed
  if (format == PointCloudLoader::Format::kAutoDetect) {
    format = PointCloudLoader::DetectFormat(filename);
  }
  
  // Analyze fields to determine optimal loading strategy
  auto analysis_metadata = PointCloudLoader::AnalyzeFields(filename, format);
  std::string optimal_type = analysis_metadata.GetRecommendedPCLType();
  
  // Load and convert based on detected type
  if (optimal_type == "PointXYZRGB") {
    auto [pcl_cloud, metadata] = PointCloudLoader::LoadToPCL<pcl::PointXYZRGB>(
        filename, format, progress_callback);
    auto renderer_data = ConvertRGBCloud(*pcl_cloud);
    return {std::move(renderer_data), std::move(metadata)};
    
  } else if (optimal_type == "PointXYZRGBA") {
    auto [pcl_cloud, metadata] = PointCloudLoader::LoadToPCL<pcl::PointXYZRGBA>(
        filename, format, progress_callback);
    auto renderer_data = ConvertRGBCloud(*pcl_cloud);
    return {std::move(renderer_data), std::move(metadata)};
    
  } else if (optimal_type == "PointXYZI") {
    auto [pcl_cloud, metadata] = PointCloudLoader::LoadToPCL<pcl::PointXYZI>(
        filename, format, progress_callback);
    auto renderer_data = ConvertIntensityCloud(*pcl_cloud);
    return {std::move(renderer_data), std::move(metadata)};
    
  } else {
    // Default to PointXYZ
    auto [pcl_cloud, metadata] = PointCloudLoader::LoadToPCL<pcl::PointXYZ>(
        filename, format, progress_callback);
    auto renderer_data = ConvertXYZCloud(*pcl_cloud, metadata);
    return {std::move(renderer_data), std::move(metadata)};
  }
}

template<typename PCLPointT>
RendererData RendererFactory::ConvertRGBCloud(const pcl::PointCloud<PCLPointT>& cloud) {
  RendererData data;
  data.color_mode = RendererData::ColorMode::kRGB;
  
  // Reserve space for efficiency
  data.points_3d.reserve(cloud.points.size());
  data.colors_rgb.reserve(cloud.points.size());
  
  // Convert points, filtering out NaN values
  for (const auto& pt : cloud.points) {
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
      data.points_3d.emplace_back(pt.x, pt.y, pt.z);
      data.colors_rgb.emplace_back(pt.r / 255.0f, pt.g / 255.0f, pt.b / 255.0f);
    }
  }
  
  return data;
}

template<typename PCLPointT>
RendererData RendererFactory::ConvertIntensityCloud(const pcl::PointCloud<PCLPointT>& cloud) {
  RendererData data;
  data.color_mode = RendererData::ColorMode::kIntensity;
  
  // Reserve space
  data.points_4d.reserve(cloud.points.size());
  
  // First pass: find intensity range
  float min_intensity = std::numeric_limits<float>::max();
  float max_intensity = std::numeric_limits<float>::lowest();
  
  for (const auto& pt : cloud.points) {
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) && !std::isnan(pt.intensity)) {
      min_intensity = std::min(min_intensity, pt.intensity);
      max_intensity = std::max(max_intensity, pt.intensity);
    }
  }
  
  // Handle edge case of constant intensity
  float intensity_range = max_intensity - min_intensity;
  if (intensity_range < 0.001f) {
    intensity_range = 1.0f;
    min_intensity = 0.0f;
  }
  
  data.scalar_range = glm::vec2(0.0f, 1.0f); // Normalized range
  
  // Second pass: convert and normalize
  for (const auto& pt : cloud.points) {
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
      float normalized_intensity = std::isnan(pt.intensity) ? 0.0f : 
                                  (pt.intensity - min_intensity) / intensity_range;
      data.points_4d.emplace_back(pt.x, pt.y, pt.z, normalized_intensity);
    }
  }
  
  return data;
}

template<typename PCLPointT>
RendererData RendererFactory::ConvertXYZCloud(const pcl::PointCloud<PCLPointT>& cloud,
                                              const PointCloudMetadata& metadata) {
  RendererData data;
  data.color_mode = RendererData::ColorMode::kHeight;
  
  // Use Z coordinate for height-based coloring
  data.scalar_range = glm::vec2(metadata.min_bounds.z, metadata.max_bounds.z);
  
  // Reserve space
  data.points_4d.reserve(cloud.points.size());
  
  // Convert points, using Z as scalar value
  for (const auto& pt : cloud.points) {
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
      data.points_4d.emplace_back(pt.x, pt.y, pt.z, pt.z);
    }
  }
  
  return data;
}

// FactoryRegistry implementation
std::unique_ptr<RendererFactory> FactoryRegistry::CreateRendererFactory() {
  return std::make_unique<RendererFactory>();
}

std::pair<RendererData, PointCloudMetadata>
FactoryRegistry::LoadForRenderer(const std::string& filename,
                                PointCloudLoader::Format format,
                                ProgressCallback progress_callback) {
  auto factory = CreateRendererFactory();
  return factory->Load(filename, format, progress_callback);
}

// Explicit template instantiations
template RendererData RendererFactory::ConvertRGBCloud<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB>&);
template RendererData RendererFactory::ConvertRGBCloud<pcl::PointXYZRGBA>(const pcl::PointCloud<pcl::PointXYZRGBA>&);
template RendererData RendererFactory::ConvertIntensityCloud<pcl::PointXYZI>(const pcl::PointCloud<pcl::PointXYZI>&);
template RendererData RendererFactory::ConvertXYZCloud<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>&, const PointCloudMetadata&);

} // namespace factory

} // namespace pcl_bridge
} // namespace quickviz