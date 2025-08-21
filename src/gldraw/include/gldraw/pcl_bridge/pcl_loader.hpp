/*
 * @file pcl_loader.hpp
 * @date Dec 2024
 * @brief Point cloud file loader with automatic field detection and PCL type selection
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_PCL_LOADER_HPP
#define QUICKVIZ_PCL_LOADER_HPP

#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <glm/glm.hpp>

// Forward declarations
namespace pcl {
template<typename PointT>
class PointCloud;
struct PointXYZ;
struct PointXYZI;
struct PointXYZRGB;
struct PointXYZRGBA;
}

namespace quickviz {
class PointCloud;
}

namespace quickviz {
namespace pcl_bridge {

/**
 * @brief Exception types for point cloud loading errors
 */
class PointCloudLoaderException : public std::runtime_error {
 public:
  explicit PointCloudLoaderException(const std::string& message) 
      : std::runtime_error(message) {}
};

class FileNotFoundException : public PointCloudLoaderException {
 public:
  explicit FileNotFoundException(const std::string& filename)
      : PointCloudLoaderException("File not found: " + filename) {}
};

class UnsupportedFormatException : public PointCloudLoaderException {
 public:
  explicit UnsupportedFormatException(const std::string& format)
      : PointCloudLoaderException("Unsupported file format: " + format) {}
};

class CorruptedFileException : public PointCloudLoaderException {
 public:
  explicit CorruptedFileException(const std::string& message)
      : PointCloudLoaderException("Corrupted file: " + message) {}
};

/**
 * @brief Detected fields in point cloud file
 */
struct PointCloudFields {
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_intensity = false;
  bool has_rgb = false;
  bool has_rgba = false;
  bool has_r = false;
  bool has_g = false;
  bool has_b = false;
  bool has_a = false;
  bool has_normal_x = false;
  bool has_normal_y = false;
  bool has_normal_z = false;
  
  // Helper methods
  bool HasXYZ() const { return has_x && has_y && has_z; }
  bool HasRGBColor() const { return has_rgb || (has_r && has_g && has_b); }
  bool HasRGBAColor() const { return has_rgba || (has_r && has_g && has_b && has_a); }
  bool HasNormals() const { return has_normal_x && has_normal_y && has_normal_z; }
};

/**
 * @brief Metadata about loaded point cloud
 */
struct PointCloudMetadata {
  std::string filename;
  std::string format;
  std::string detected_pcl_type;
  size_t point_count = 0;
  PointCloudFields fields;
  glm::vec3 min_bounds{0.0f};
  glm::vec3 max_bounds{0.0f};
  double file_size_mb = 0.0;
  
  // Get recommended PCL point type based on detected fields
  std::string GetRecommendedPCLType() const;
};

/**
 * @brief Progress callback for large file loading
 */
using ProgressCallback = std::function<void(float percentage, const std::string& message)>;

/**
 * @brief Point cloud file loader with automatic field detection
 */
class PointCloudLoader {
 public:
  enum class Format {
    kPCD,
    kPLY,
    kAutoDetect  // Detect format from file extension
  };

  /**
   * @brief Load point cloud with automatic PCL type detection and conversion
   * @param filename Path to the point cloud file
   * @param renderer_cloud Output QuickViz point cloud
   * @param format File format (kAutoDetect by default)
   * @param progress_callback Optional progress callback for large files
   * @return Metadata about the loaded point cloud
   */
  static PointCloudMetadata Load(const std::string& filename,
                                PointCloud& renderer_cloud,
                                Format format = Format::kAutoDetect,
                                ProgressCallback progress_callback = nullptr);

  /**
   * @brief Load PCD file with automatic field detection
   * @param filename Path to the PCD file
   * @param renderer_cloud Output QuickViz point cloud
   * @param progress_callback Optional progress callback
   * @return Metadata about the loaded point cloud
   */
  static PointCloudMetadata LoadPCD(const std::string& filename,
                                   PointCloud& renderer_cloud,
                                   ProgressCallback progress_callback = nullptr);

  /**
   * @brief Load PLY file with automatic field detection
   * @param filename Path to the PLY file
   * @param renderer_cloud Output QuickViz point cloud
   * @param progress_callback Optional progress callback
   * @return Metadata about the loaded point cloud
   */
  static PointCloudMetadata LoadPLY(const std::string& filename,
                                   PointCloud& renderer_cloud,
                                   ProgressCallback progress_callback = nullptr);

  /**
   * @brief Analyze file fields without loading the full point cloud
   * @param filename Path to the point cloud file
   * @param format File format (kAutoDetect by default)
   * @return Metadata with field information
   */
  static PointCloudMetadata AnalyzeFields(const std::string& filename,
                                         Format format = Format::kAutoDetect);

  /**
   * @brief Load point cloud into appropriate PCL format based on detected fields
   * @tparam PCLPointT Target PCL point type (must match detected fields)
   * @param filename Path to the point cloud file
   * @param format File format (kAutoDetect by default)
   * @param progress_callback Optional progress callback
   * @return Shared pointer to PCL point cloud and metadata
   */
  template<typename PCLPointT>
  static std::pair<typename pcl::PointCloud<PCLPointT>::Ptr, PointCloudMetadata>
  LoadToPCL(const std::string& filename,
            Format format = Format::kAutoDetect,
            ProgressCallback progress_callback = nullptr);

  /**
   * @brief Get file format from filename extension
   * @param filename Path to the file
   * @return Detected format
   */
  static Format DetectFormat(const std::string& filename);

  /**
   * @brief Check if file format is supported
   * @param format Format to check
   * @return True if supported
   */
  static bool IsFormatSupported(Format format);

  /**
   * @brief Get list of supported file extensions
   * @return Vector of supported extensions (e.g., ".pcd", ".ply")
   */
  static std::vector<std::string> GetSupportedExtensions();

 private:
  /**
   * @brief Detect fields in PCD file header
   */
  static PointCloudFields DetectPCDFields(const std::string& filename);

  /**
   * @brief Detect fields in PLY file header
   */
  static PointCloudFields DetectPLYFields(const std::string& filename);

  /**
   * @brief Load PCD file using detected optimal PCL type
   */
  static PointCloudMetadata LoadPCDWithAutoType(const std::string& filename,
                                               PointCloud& renderer_cloud,
                                               const PointCloudFields& fields,
                                               ProgressCallback progress_callback);

  /**
   * @brief Load PLY file using detected optimal PCL type
   */
  static PointCloudMetadata LoadPLYWithAutoType(const std::string& filename,
                                               PointCloud& renderer_cloud,
                                               const PointCloudFields& fields,
                                               ProgressCallback progress_callback);

  /**
   * @brief Load PCD file with specific PCL point type
   */
  template<typename PCLPointT>
  static std::pair<typename pcl::PointCloud<PCLPointT>::Ptr, PointCloudMetadata>
  LoadPCDInternal(const std::string& filename, ProgressCallback progress_callback);

  /**
   * @brief Load PLY file with specific PCL point type
   */
  template<typename PCLPointT>
  static std::pair<typename pcl::PointCloud<PCLPointT>::Ptr, PointCloudMetadata>
  LoadPLYInternal(const std::string& filename, ProgressCallback progress_callback);

  /**
   * @brief Calculate metadata from PCL point cloud
   */
  template<typename PCLPointT>
  static PointCloudMetadata CalculateMetadata(
      const std::string& filename,
      const std::string& format,
      const pcl::PointCloud<PCLPointT>& cloud,
      const PointCloudFields& fields);

  /**
   * @brief Validate file exists and is readable
   */
  static void ValidateFileAccess(const std::string& filename);

  /**
   * @brief Get file size in MB
   */
  static double GetFileSizeMB(const std::string& filename);

  /**
   * @brief Convert format enum to string
   */
  static std::string FormatToString(Format format);

  /**
   * @brief Get point type string for PCL point type
   */
  template<typename PCLPointT>
  static std::string GetPointTypeString();

  /**
   * @brief Determine optimal PCL type from detected fields
   */
  static std::string DetermineOptimalPCLType(const PointCloudFields& fields);
};

/**
 * @brief Utility functions for field detection and type matching
 */
namespace field_detector {

/**
 * @brief Parse PCD header to extract field information
 * @param filename Path to PCD file
 * @return Detected fields structure
 */
PointCloudFields ParsePCDHeader(const std::string& filename);

/**
 * @brief Parse PLY header to extract field information
 * @param filename Path to PLY file
 * @return Detected fields structure
 */
PointCloudFields ParsePLYHeader(const std::string& filename);

/**
 * @brief Check if PCL point type is compatible with detected fields
 * @tparam PCLPointT PCL point type to check
 * @param fields Detected fields
 * @return True if compatible
 */
template<typename PCLPointT>
bool IsCompatiblePCLType(const PointCloudFields& fields);

/**
 * @brief Get field requirements for specific PCL point types
 */
PointCloudFields GetRequiredFields(const std::string& pcl_type);

} // namespace field_detector

/**
 * @brief Factory pattern for creating different point cloud representations
 */
namespace factory {

/**
 * @brief Renderer-optimized point cloud data structure
 */
struct RendererData {
  std::vector<glm::vec3> points_3d;      // XYZ coordinates
  std::vector<glm::vec3> colors_rgb;     // RGB colors (if available)
  std::vector<glm::vec4> points_4d;      // XYZI coordinates (if intensity/scalar available)
  
  enum class ColorMode {
    kRGB,           // Use RGB colors
    kIntensity,     // Use intensity field
    kHeight,        // Use Z coordinate for height coloring
    kScalar         // Use W component as scalar
  } color_mode = ColorMode::kHeight;
  
  glm::vec2 scalar_range{0.0f, 1.0f};    // Min/max range for scalar coloring
  
  bool HasRGBColors() const { return !colors_rgb.empty(); }
  bool HasScalarData() const { return !points_4d.empty(); }
  size_t GetPointCount() const { 
    return HasRGBColors() ? points_3d.size() : points_4d.size(); 
  }
};

/**
 * @brief Base factory interface for point cloud conversion
 */
template<typename OutputType>
class PointCloudFactory {
public:
  virtual ~PointCloudFactory() = default;
  
  /**
   * @brief Load and convert point cloud to target format
   * @param filename Path to point cloud file
   * @param format File format (auto-detect if not specified)
   * @param progress_callback Optional progress reporting
   * @return Converted data and metadata
   */
  virtual std::pair<OutputType, PointCloudMetadata> 
  Load(const std::string& filename,
       PointCloudLoader::Format format = PointCloudLoader::Format::kAutoDetect,
       ProgressCallback progress_callback = nullptr) = 0;
};

/**
 * @brief Factory for creating renderer-optimized point clouds
 */
class RendererFactory : public PointCloudFactory<RendererData> {
public:
  /**
   * @brief Create renderer-optimized point cloud data
   * @param filename Path to point cloud file
   * @param format File format (auto-detect if not specified)  
   * @param progress_callback Optional progress reporting
   * @return RendererData with optimized format and metadata
   */
  std::pair<RendererData, PointCloudMetadata> 
  Load(const std::string& filename,
       PointCloudLoader::Format format = PointCloudLoader::Format::kAutoDetect,
       ProgressCallback progress_callback = nullptr) override;

private:
  /**
   * @brief Convert PCL RGB point cloud to renderer format
   */
  template<typename PCLPointT>
  RendererData ConvertRGBCloud(const pcl::PointCloud<PCLPointT>& cloud);
  
  /**
   * @brief Convert PCL intensity point cloud to renderer format  
   */
  template<typename PCLPointT>
  RendererData ConvertIntensityCloud(const pcl::PointCloud<PCLPointT>& cloud);
  
  /**
   * @brief Convert PCL XYZ point cloud to renderer format
   */
  template<typename PCLPointT>
  RendererData ConvertXYZCloud(const pcl::PointCloud<PCLPointT>& cloud,
                                const PointCloudMetadata& metadata);
};

/**
 * @brief Factory registry for different output formats
 */
class FactoryRegistry {
public:
  /**
   * @brief Get renderer factory instance
   */
  static std::unique_ptr<RendererFactory> CreateRendererFactory();
  
  /**
   * @brief Convenience method for direct renderer loading
   * @param filename Path to point cloud file
   * @param format File format (auto-detect if not specified)
   * @param progress_callback Optional progress reporting
   * @return RendererData and metadata
   */
  static std::pair<RendererData, PointCloudMetadata>
  LoadForRenderer(const std::string& filename,
                  PointCloudLoader::Format format = PointCloudLoader::Format::kAutoDetect,
                  ProgressCallback progress_callback = nullptr);
};

} // namespace factory

} // namespace pcl_bridge
} // namespace quickviz

#endif // QUICKVIZ_PCL_LOADER_HPP