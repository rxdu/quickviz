/**
 * @file occupancy_grid.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Occupancy grid renderer for maps and probabilistic grids
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_OCCUPANCY_GRID_HPP
#define QUICKVIZ_OCCUPANCY_GRID_HPP

#include <vector>
#include <memory>
#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Renderable 2D/3D occupancy grid for mapping and SLAM
 * 
 * Used for visualizing:
 * - SLAM maps and occupancy grids
 * - Probabilistic occupancy maps
 * - Cost maps for path planning
 * - Height maps and terrain data
 * - Sensor coverage maps
 * - Multi-layer semantic maps
 */
class OccupancyGrid : public OpenGlObject {
 public:
  enum class RenderMode {
    kFlat2D,          // Flat 2D grid at fixed height
    kHeightmap,       // 3D height-based visualization
    kVoxels,          // 3D volumetric representation
    kContour          // Contour lines for elevation
  };

  enum class ColorMode {
    kOccupancy,       // Black/white for occupied/free
    kProbability,     // Grayscale for probability values
    kCostmap,         // Blue to red for cost values
    kHeight,          // Rainbow for height values
    kSemantic,        // Custom colors for semantic classes
    kCustom           // User-defined color mapping
  };

  enum class CellShape {
    kSquare,          // Square cells (fastest)
    kCircle,          // Circular cells for smooth appearance
    kHexagon          // Hexagonal cells for alternative layouts
  };

  OccupancyGrid();
  OccupancyGrid(size_t width, size_t height, float resolution);
  ~OccupancyGrid();

  // Grid data management
  void SetGridSize(size_t width, size_t height);
  void SetResolution(float resolution);  // Meters per cell
  void SetOrigin(const glm::vec3& origin);  // World position of grid corner
  void SetData(const std::vector<float>& data);  // Row-major order
  void SetData(const std::vector<int8_t>& data);  // ROS-style occupancy data
  void SetCell(size_t x, size_t y, float value);
  void Clear();
  
  // Multi-layer support
  void SetLayerCount(size_t layers);
  void SetLayerData(size_t layer, const std::vector<float>& data);
  void SetLayerHeight(size_t layer, float height);
  void SetLayerOpacity(size_t layer, float alpha);
  
  // Rendering configuration
  void SetRenderMode(RenderMode mode);
  void SetColorMode(ColorMode mode);
  void SetCellShape(CellShape shape);
  void SetValueRange(const glm::vec2& min_max);  // For color mapping
  void SetHeightScale(float scale);              // For heightmap mode
  void SetMaxHeight(float max_height);           // Clamp heights
  
  // Color customization
  void SetOccupiedColor(const glm::vec3& color);
  void SetFreeColor(const glm::vec3& color);
  void SetUnknownColor(const glm::vec3& color);
  void SetColorMap(const std::vector<glm::vec3>& colors);  // For custom mapping
  
  // Visual options
  void SetShowGrid(bool show);
  void SetGridColor(const glm::vec3& color);
  void SetGridLineWidth(float width);
  void SetTransparency(float alpha);
  void SetBorderWidth(float width);
  void SetBorderColor(const glm::vec3& color);
  
  // Filtering and LOD
  void SetValueThreshold(float threshold);       // Hide cells below threshold
  void SetLevelOfDetail(bool enable, float distance_threshold = 20.0f);
  void SetSubsampling(size_t factor);            // Render every Nth cell
  void SetSmoothInterpolation(bool smooth);      // Bilinear interpolation
  
  // Animation and updates
  void SetAnimationTime(float time);             // For time-varying grids
  void UpdateRegion(size_t x_start, size_t y_start, size_t width, size_t height);
  
  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_grid_ != 0; }

  // Utility methods
  size_t GetWidth() const { return width_; }
  size_t GetHeight() const { return height_; }
  float GetResolution() const { return resolution_; }
  glm::vec3 GetOrigin() const { return origin_; }
  float GetCell(size_t x, size_t y) const;
  
  // World/grid coordinate conversion
  glm::vec2 WorldToGrid(const glm::vec3& world_pos) const;
  glm::vec3 GridToWorld(size_t x, size_t y) const;
  void GetBoundingBox(glm::vec3& min_corner, glm::vec3& max_corner) const;
  
  // Statistical queries
  float GetMinValue() const;
  float GetMaxValue() const;
  float GetOccupancyRatio() const;  // Fraction of occupied cells
  size_t GetOccupiedCellCount() const;
  
 private:
  void GenerateGridGeometry();
  void GenerateHeightmapGeometry();
  void GenerateVoxelGeometry();
  void GenerateContourGeometry();
  
  void GenerateQuadCell(size_t x, size_t y, float value, std::vector<glm::vec3>& vertices,
                       std::vector<glm::vec3>& colors, std::vector<uint32_t>& indices);
  void GenerateCircleCell(size_t x, size_t y, float value, std::vector<glm::vec3>& vertices,
                         std::vector<glm::vec3>& colors, std::vector<uint32_t>& indices);
  void GenerateHexagonCell(size_t x, size_t y, float value, std::vector<glm::vec3>& vertices,
                          std::vector<glm::vec3>& colors, std::vector<uint32_t>& indices);
  void GenerateVoxelCell(size_t x, size_t y, float value, size_t layer, float layer_height, float layer_opacity);
  
  glm::vec3 ComputeCellColor(float value, size_t layer = 0) const;
  float ComputeCellHeight(float value) const;
  bool ShouldRenderCell(float value) const;
  
  void UpdateGpuBuffers();
  void UpdateGridBuffers();  // For grid lines
  void UpdateBorderBuffers(); // For border
  
  // Grid parameters
  size_t width_ = 100;
  size_t height_ = 100;
  float resolution_ = 0.1f;  // meters per cell
  glm::vec3 origin_ = glm::vec3(0.0f);
  
  // Data storage
  std::vector<float> data_;
  std::vector<std::vector<float>> layer_data_;
  std::vector<float> layer_heights_;
  std::vector<float> layer_opacities_;
  size_t layer_count_ = 1;
  
  // Rendering settings
  RenderMode render_mode_ = RenderMode::kFlat2D;
  ColorMode color_mode_ = ColorMode::kOccupancy;
  CellShape cell_shape_ = CellShape::kSquare;
  glm::vec2 value_range_ = glm::vec2(0.0f, 1.0f);
  float height_scale_ = 1.0f;
  float max_height_ = 2.0f;
  
  // Colors
  glm::vec3 occupied_color_ = glm::vec3(0.0f, 0.0f, 0.0f);      // Black
  glm::vec3 free_color_ = glm::vec3(1.0f, 1.0f, 1.0f);         // White
  glm::vec3 unknown_color_ = glm::vec3(0.5f, 0.5f, 0.5f);      // Gray
  std::vector<glm::vec3> custom_colors_;
  
  // Visual options
  bool show_grid_ = false;
  glm::vec3 grid_color_ = glm::vec3(0.7f, 0.7f, 0.7f);
  float grid_line_width_ = 1.0f;
  float transparency_ = 1.0f;
  float border_width_ = 0.0f;
  glm::vec3 border_color_ = glm::vec3(0.0f, 0.0f, 0.0f);
  
  // Performance options
  float value_threshold_ = -1.0f;  // Disabled by default
  bool level_of_detail_ = false;
  float lod_distance_threshold_ = 20.0f;
  size_t subsampling_factor_ = 1;
  bool smooth_interpolation_ = false;
  
  // Animation
  float animation_time_ = 0.0f;
  
  // Geometry data
  std::vector<glm::vec3> cell_vertices_;
  std::vector<glm::vec3> cell_colors_;
  std::vector<glm::vec2> cell_texcoords_;  // For texture-based rendering
  std::vector<uint32_t> cell_indices_;
  
  std::vector<glm::vec3> grid_vertices_;
  std::vector<glm::vec3> border_vertices_;
  
  // OpenGL resources
  uint32_t vao_grid_ = 0;
  uint32_t vbo_vertices_ = 0;
  uint32_t vbo_colors_ = 0;
  uint32_t vbo_texcoords_ = 0;
  uint32_t ebo_indices_ = 0;
  
  uint32_t vao_lines_ = 0;
  uint32_t vbo_grid_lines_ = 0;
  uint32_t vbo_border_lines_ = 0;
  
  // Texture for large grids (optional)
  uint32_t grid_texture_ = 0;
  
  // Shaders
  ShaderProgram cell_shader_;
  ShaderProgram line_shader_;
  ShaderProgram texture_shader_;  // For texture-based rendering
  
  bool needs_update_ = true;
  bool needs_grid_update_ = true;
  bool needs_border_update_ = true;
};

}  // namespace quickviz

#endif  // QUICKVIZ_OCCUPANCY_GRID_HPP