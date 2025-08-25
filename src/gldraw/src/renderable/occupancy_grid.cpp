/**
 * @file occupancy_grid.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Occupancy grid renderer implementation
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/renderable/occupancy_grid.hpp"

#ifdef IMVIEW_WITH_GLAD
#include <glad/glad.h>
#else
#include <GL/gl.h>
#endif

#include <algorithm>
#include <cmath>
#include <iostream>

#include "gldraw/shader.hpp"

namespace quickviz {

namespace {

// Vertex shader for cells
constexpr const char* kCellVertexShader = R"(
#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
layout (location = 2) in vec2 aTexCoord;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uCoordTransform;
uniform float uTransparency;

out vec3 FragColor;
out vec2 TexCoord;
out float Alpha;

void main() {
    FragColor = aColor;
    TexCoord = aTexCoord;
    Alpha = uTransparency;
    
    vec4 worldPos = uCoordTransform * vec4(aPos, 1.0);
    gl_Position = uProjection * uView * worldPos;
}
)";

// Fragment shader for cells
constexpr const char* kCellFragmentShader = R"(
#version 330 core

in vec3 FragColor;
in vec2 TexCoord;
in float Alpha;

out vec4 color;

void main() {
    color = vec4(FragColor, Alpha);
}
)";

// Vertex shader for grid lines
constexpr const char* kLineVertexShader = R"(
#version 330 core

layout (location = 0) in vec3 aPos;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uCoordTransform;

void main() {
    vec4 worldPos = uCoordTransform * vec4(aPos, 1.0);
    gl_Position = uProjection * uView * worldPos;
}
)";

// Fragment shader for grid lines
constexpr const char* kLineFragmentShader = R"(
#version 330 core

uniform vec3 uColor;
uniform float uAlpha;

out vec4 color;

void main() {
    color = vec4(uColor, uAlpha);
}
)";

}  // namespace

OccupancyGrid::OccupancyGrid() {
  data_.resize(width_ * height_, -1.0f);  // Initialize with unknown values
  layer_data_.resize(layer_count_);
  layer_heights_.resize(layer_count_, 0.0f);
  layer_opacities_.resize(layer_count_, 1.0f);
}

OccupancyGrid::OccupancyGrid(size_t width, size_t height, float resolution)
    : width_(width), height_(height), resolution_(resolution) {
  data_.resize(width_ * height_, -1.0f);
  layer_data_.resize(layer_count_);
  layer_heights_.resize(layer_count_, 0.0f);
  layer_opacities_.resize(layer_count_, 1.0f);
}

OccupancyGrid::~OccupancyGrid() { ReleaseGpuResources(); }

void OccupancyGrid::SetGridSize(size_t width, size_t height) {
  width_ = width;
  height_ = height;
  data_.resize(width_ * height_, -1.0f);
  for (auto& layer : layer_data_) {
    layer.resize(width_ * height_, -1.0f);
  }
  needs_update_ = true;
}

void OccupancyGrid::SetResolution(float resolution) {
  resolution_ = resolution;
  needs_update_ = true;
}

void OccupancyGrid::SetOrigin(const glm::vec3& origin) {
  origin_ = origin;
  needs_update_ = true;
}

void OccupancyGrid::SetData(const std::vector<float>& data) {
  if (data.size() != width_ * height_) {
    std::cerr << "OccupancyGrid: Data size mismatch. Expected "
              << width_ * height_ << ", got " << data.size() << std::endl;
    return;
  }
  data_ = data;
  needs_update_ = true;
}

void OccupancyGrid::SetData(const std::vector<int8_t>& data) {
  if (data.size() != width_ * height_) {
    std::cerr << "OccupancyGrid: Data size mismatch. Expected "
              << width_ * height_ << ", got " << data.size() << std::endl;
    return;
  }

  data_.resize(data.size());
  for (size_t i = 0; i < data.size(); ++i) {
    if (data[i] == -1) {
      data_[i] = -1.0f;  // Unknown
    } else {
      data_[i] =
          static_cast<float>(data[i]) / 100.0f;  // Convert 0-100 to 0.0-1.0
    }
  }
  needs_update_ = true;
}

void OccupancyGrid::SetCell(size_t x, size_t y, float value) {
  if (x >= width_ || y >= height_) return;
  data_[y * width_ + x] = value;
  needs_update_ = true;
}

void OccupancyGrid::Clear() {
  std::fill(data_.begin(), data_.end(), -1.0f);
  for (auto& layer : layer_data_) {
    std::fill(layer.begin(), layer.end(), -1.0f);
  }
  needs_update_ = true;
}

void OccupancyGrid::SetLayerCount(size_t layers) {
  layer_count_ = layers;
  layer_data_.resize(layer_count_);
  layer_heights_.resize(layer_count_, 0.0f);
  layer_opacities_.resize(layer_count_, 1.0f);
  for (auto& layer : layer_data_) {
    layer.resize(width_ * height_, -1.0f);
  }
  needs_update_ = true;
}

void OccupancyGrid::SetLayerData(size_t layer, const std::vector<float>& data) {
  if (layer >= layer_count_) return;
  if (data.size() != width_ * height_) {
    std::cerr << "OccupancyGrid: Layer data size mismatch" << std::endl;
    return;
  }
  layer_data_[layer] = data;
  needs_update_ = true;
}

void OccupancyGrid::SetLayerHeight(size_t layer, float height) {
  if (layer >= layer_count_) return;
  layer_heights_[layer] = height;
  needs_update_ = true;
}

void OccupancyGrid::SetLayerOpacity(size_t layer, float alpha) {
  if (layer >= layer_count_) return;
  layer_opacities_[layer] = glm::clamp(alpha, 0.0f, 1.0f);
  needs_update_ = true;
}

void OccupancyGrid::SetRenderMode(RenderMode mode) {
  render_mode_ = mode;
  needs_update_ = true;
}

void OccupancyGrid::SetColorMode(ColorMode mode) {
  color_mode_ = mode;
  needs_update_ = true;
}

void OccupancyGrid::SetCellShape(CellShape shape) {
  cell_shape_ = shape;
  needs_update_ = true;
}

void OccupancyGrid::SetValueRange(const glm::vec2& min_max) {
  value_range_ = min_max;
  needs_update_ = true;
}

void OccupancyGrid::SetHeightScale(float scale) {
  height_scale_ = scale;
  needs_update_ = true;
}

void OccupancyGrid::SetMaxHeight(float max_height) {
  max_height_ = max_height;
  needs_update_ = true;
}

void OccupancyGrid::SetOccupiedColor(const glm::vec3& color) {
  occupied_color_ = color;
  needs_update_ = true;
}

void OccupancyGrid::SetFreeColor(const glm::vec3& color) {
  free_color_ = color;
  needs_update_ = true;
}

void OccupancyGrid::SetUnknownColor(const glm::vec3& color) {
  unknown_color_ = color;
  needs_update_ = true;
}

void OccupancyGrid::SetColorMap(const std::vector<glm::vec3>& colors) {
  custom_colors_ = colors;
  if (color_mode_ == ColorMode::kCustom) {
    needs_update_ = true;
  }
}

void OccupancyGrid::SetShowGrid(bool show) {
  show_grid_ = show;
  needs_grid_update_ = true;
}

void OccupancyGrid::SetGridColor(const glm::vec3& color) {
  grid_color_ = color;
}

void OccupancyGrid::SetGridLineWidth(float width) { grid_line_width_ = width; }

void OccupancyGrid::SetTransparency(float alpha) {
  transparency_ = glm::clamp(alpha, 0.0f, 1.0f);
}

void OccupancyGrid::SetBorderWidth(float width) {
  border_width_ = width;
  needs_border_update_ = true;
}

void OccupancyGrid::SetBorderColor(const glm::vec3& color) {
  border_color_ = color;
}

void OccupancyGrid::SetValueThreshold(float threshold) {
  value_threshold_ = threshold;
  needs_update_ = true;
}

void OccupancyGrid::SetLevelOfDetail(bool enable, float distance_threshold) {
  level_of_detail_ = enable;
  lod_distance_threshold_ = distance_threshold;
}

void OccupancyGrid::SetSubsampling(size_t factor) {
  subsampling_factor_ = std::max(size_t(1), factor);
  needs_update_ = true;
}

void OccupancyGrid::SetSmoothInterpolation(bool smooth) {
  smooth_interpolation_ = smooth;
  needs_update_ = true;
}

void OccupancyGrid::AllocateGpuResources() {
  if (IsGpuResourcesAllocated()) return;

  // Create cell shader
  try {
    Shader cell_vs(kCellVertexShader, Shader::Type::kVertex);
    Shader cell_fs(kCellFragmentShader, Shader::Type::kFragment);
    cell_shader_.AttachShader(cell_vs);
    cell_shader_.AttachShader(cell_fs);

    if (!cell_shader_.LinkProgram()) {
      std::cerr << "OccupancyGrid: Failed to link cell shader program"
                << std::endl;
      return;
    }
  } catch (const std::exception& e) {
    std::cerr << "OccupancyGrid: Failed to create cell shader: " << e.what()
              << std::endl;
    return;
  }

  // Create line shader
  try {
    Shader line_vs(kLineVertexShader, Shader::Type::kVertex);
    Shader line_fs(kLineFragmentShader, Shader::Type::kFragment);
    line_shader_.AttachShader(line_vs);
    line_shader_.AttachShader(line_fs);

    if (!line_shader_.LinkProgram()) {
      std::cerr << "OccupancyGrid: Failed to link line shader program"
                << std::endl;
      return;
    }
  } catch (const std::exception& e) {
    std::cerr << "OccupancyGrid: Failed to create line shader: " << e.what()
              << std::endl;
    return;
  }

  // Generate cell buffers
  glGenVertexArrays(1, &vao_grid_);
  glGenBuffers(1, &vbo_vertices_);
  glGenBuffers(1, &vbo_colors_);
  glGenBuffers(1, &vbo_texcoords_);
  glGenBuffers(1, &ebo_indices_);

  // Generate line buffers
  glGenVertexArrays(1, &vao_lines_);
  glGenBuffers(1, &vbo_grid_lines_);
  glGenBuffers(1, &vbo_border_lines_);

  needs_update_ = true;
  needs_grid_update_ = true;
  needs_border_update_ = true;
}

void OccupancyGrid::ReleaseGpuResources() noexcept {
  if (vao_grid_ != 0) {
    glDeleteVertexArrays(1, &vao_grid_);
    glDeleteBuffers(1, &vbo_vertices_);
    glDeleteBuffers(1, &vbo_colors_);
    glDeleteBuffers(1, &vbo_texcoords_);
    glDeleteBuffers(1, &ebo_indices_);
    vao_grid_ = 0;
  }

  if (vao_lines_ != 0) {
    glDeleteVertexArrays(1, &vao_lines_);
    glDeleteBuffers(1, &vbo_grid_lines_);
    glDeleteBuffers(1, &vbo_border_lines_);
    vao_lines_ = 0;
  }

  if (grid_texture_ != 0) {
    glDeleteTextures(1, &grid_texture_);
    grid_texture_ = 0;
  }
}

void OccupancyGrid::OnDraw(const glm::mat4& projection, const glm::mat4& view,
                           const glm::mat4& coord_transform) {
  if (!IsGpuResourcesAllocated()) {
    AllocateGpuResources();
  }

  if (data_.empty()) return;

  if (needs_update_) {
    switch (render_mode_) {
      case RenderMode::kFlat2D:
        GenerateGridGeometry();
        break;
      case RenderMode::kHeightmap:
        GenerateHeightmapGeometry();
        break;
      case RenderMode::kVoxels:
        GenerateVoxelGeometry();
        break;
      case RenderMode::kContour:
        GenerateContourGeometry();
        break;
    }
    UpdateGpuBuffers();
    needs_update_ = false;
  }

  if (needs_grid_update_ && show_grid_) {
    UpdateGridBuffers();
    needs_grid_update_ = false;
  }

  if (needs_border_update_ && border_width_ > 0.0f) {
    UpdateBorderBuffers();
    needs_border_update_ = false;
  }

  // Enable transparency if needed
  if (transparency_ < 1.0f) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }

  // Draw cells
  if (!cell_vertices_.empty()) {
    cell_shader_.Use();
    cell_shader_.SetUniform("uProjection", projection);
    cell_shader_.SetUniform("uView", view);
    cell_shader_.SetUniform("uCoordTransform", coord_transform);
    cell_shader_.SetUniform("uTransparency", transparency_);

    glBindVertexArray(vao_grid_);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(cell_indices_.size()),
                   GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
  }

  // Draw grid lines
  if (show_grid_ && !grid_vertices_.empty()) {
    glLineWidth(grid_line_width_);

    line_shader_.Use();
    line_shader_.SetUniform("uProjection", projection);
    line_shader_.SetUniform("uView", view);
    line_shader_.SetUniform("uCoordTransform", coord_transform);
    line_shader_.SetUniform("uColor", grid_color_);
    line_shader_.SetUniform("uAlpha", 1.0f);

    glBindVertexArray(vao_lines_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_grid_lines_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (void*)0);
    glEnableVertexAttribArray(0);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(grid_vertices_.size()));
    glBindVertexArray(0);
  }

  // Draw border
  if (border_width_ > 0.0f && !border_vertices_.empty()) {
    glLineWidth(border_width_);

    line_shader_.Use();
    line_shader_.SetUniform("uColor", border_color_);
    line_shader_.SetUniform("uAlpha", 1.0f);

    glBindVertexArray(vao_lines_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_border_lines_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (void*)0);
    glEnableVertexAttribArray(0);
    glDrawArrays(GL_LINE_LOOP, 0,
                 static_cast<GLsizei>(border_vertices_.size()));
    glBindVertexArray(0);
  }

  if (transparency_ < 1.0f) {
    glDisable(GL_BLEND);
  }
}

float OccupancyGrid::GetCell(size_t x, size_t y) const {
  if (x >= width_ || y >= height_) return -1.0f;
  return data_[y * width_ + x];
}

glm::vec2 OccupancyGrid::WorldToGrid(const glm::vec3& world_pos) const {
  glm::vec3 rel_pos = world_pos - origin_;
  return glm::vec2(rel_pos.x / resolution_, rel_pos.y / resolution_);
}

glm::vec3 OccupancyGrid::GridToWorld(size_t x, size_t y) const {
  return origin_ + glm::vec3(x * resolution_, y * resolution_, 0.0f);
}

void OccupancyGrid::GetBoundingBox(glm::vec3& min_corner,
                                   glm::vec3& max_corner) const {
  min_corner = origin_;
  max_corner = origin_ + glm::vec3(width_ * resolution_, height_ * resolution_,
                                   max_height_);
}

float OccupancyGrid::GetMinValue() const {
  if (data_.empty()) return 0.0f;
  auto it = std::min_element(data_.begin(), data_.end());
  return *it;
}

float OccupancyGrid::GetMaxValue() const {
  if (data_.empty()) return 0.0f;
  auto it = std::max_element(data_.begin(), data_.end());
  return *it;
}

float OccupancyGrid::GetOccupancyRatio() const {
  if (data_.empty()) return 0.0f;
  size_t occupied = std::count_if(data_.begin(), data_.end(),
                                  [](float val) { return val > 0.5f; });
  return static_cast<float>(occupied) / data_.size();
}

size_t OccupancyGrid::GetOccupiedCellCount() const {
  return std::count_if(data_.begin(), data_.end(),
                       [](float val) { return val > 0.5f; });
}

void OccupancyGrid::GenerateGridGeometry() {
  cell_vertices_.clear();
  cell_colors_.clear();
  cell_texcoords_.clear();
  cell_indices_.clear();

  for (size_t y = 0; y < height_; y += subsampling_factor_) {
    for (size_t x = 0; x < width_; x += subsampling_factor_) {
      float value = GetCell(x, y);
      if (!ShouldRenderCell(value)) continue;

      switch (cell_shape_) {
        case CellShape::kSquare:
          GenerateQuadCell(x, y, value, cell_vertices_, cell_colors_,
                           cell_indices_);
          break;
        case CellShape::kCircle:
          GenerateCircleCell(x, y, value, cell_vertices_, cell_colors_,
                             cell_indices_);
          break;
        case CellShape::kHexagon:
          GenerateHexagonCell(x, y, value, cell_vertices_, cell_colors_,
                              cell_indices_);
          break;
      }
    }
  }
}

void OccupancyGrid::GenerateHeightmapGeometry() {
  // Similar to GenerateGridGeometry but with Z-coordinates based on height
  cell_vertices_.clear();
  cell_colors_.clear();
  cell_texcoords_.clear();
  cell_indices_.clear();

  for (size_t y = 0; y < height_; y += subsampling_factor_) {
    for (size_t x = 0; x < width_; x += subsampling_factor_) {
      float value = GetCell(x, y);
      if (!ShouldRenderCell(value)) continue;

      float height = ComputeCellHeight(value);
      glm::vec3 color = ComputeCellColor(value);

      // Generate heightmap quad
      glm::vec3 base =
          origin_ + glm::vec3(x * resolution_, y * resolution_, 0.0f);
      size_t base_idx = cell_vertices_.size();

      // Bottom vertices (at ground level)
      cell_vertices_.push_back(base);
      cell_vertices_.push_back(base + glm::vec3(resolution_, 0.0f, 0.0f));
      cell_vertices_.push_back(base +
                               glm::vec3(resolution_, resolution_, 0.0f));
      cell_vertices_.push_back(base + glm::vec3(0.0f, resolution_, 0.0f));

      // Top vertices (at height level)
      cell_vertices_.push_back(base + glm::vec3(0.0f, 0.0f, height));
      cell_vertices_.push_back(base + glm::vec3(resolution_, 0.0f, height));
      cell_vertices_.push_back(base +
                               glm::vec3(resolution_, resolution_, height));
      cell_vertices_.push_back(base + glm::vec3(0.0f, resolution_, height));

      // Colors for all vertices
      for (int i = 0; i < 8; ++i) {
        cell_colors_.push_back(color);
        cell_texcoords_.push_back(
            glm::vec2(0.0f, 0.0f));  // Basic texture coordinates
      }

      // Generate indices for the box (6 faces)
      // Bottom face
      cell_indices_.insert(cell_indices_.end(),
                           {static_cast<uint32_t>(base_idx + 0),
                            static_cast<uint32_t>(base_idx + 1),
                            static_cast<uint32_t>(base_idx + 2),
                            static_cast<uint32_t>(base_idx + 0),
                            static_cast<uint32_t>(base_idx + 2),
                            static_cast<uint32_t>(base_idx + 3)});
      // Top face
      cell_indices_.insert(cell_indices_.end(),
                           {static_cast<uint32_t>(base_idx + 4),
                            static_cast<uint32_t>(base_idx + 6),
                            static_cast<uint32_t>(base_idx + 5),
                            static_cast<uint32_t>(base_idx + 4),
                            static_cast<uint32_t>(base_idx + 7),
                            static_cast<uint32_t>(base_idx + 6)});
      // Side faces
      cell_indices_.insert(cell_indices_.end(),
                           {static_cast<uint32_t>(base_idx + 0),
                            static_cast<uint32_t>(base_idx + 4),
                            static_cast<uint32_t>(base_idx + 5),
                            static_cast<uint32_t>(base_idx + 0),
                            static_cast<uint32_t>(base_idx + 5),
                            static_cast<uint32_t>(base_idx + 1),
                            static_cast<uint32_t>(base_idx + 1),
                            static_cast<uint32_t>(base_idx + 5),
                            static_cast<uint32_t>(base_idx + 6),
                            static_cast<uint32_t>(base_idx + 1),
                            static_cast<uint32_t>(base_idx + 6),
                            static_cast<uint32_t>(base_idx + 2),
                            static_cast<uint32_t>(base_idx + 2),
                            static_cast<uint32_t>(base_idx + 6),
                            static_cast<uint32_t>(base_idx + 7),
                            static_cast<uint32_t>(base_idx + 2),
                            static_cast<uint32_t>(base_idx + 7),
                            static_cast<uint32_t>(base_idx + 3),
                            static_cast<uint32_t>(base_idx + 3),
                            static_cast<uint32_t>(base_idx + 7),
                            static_cast<uint32_t>(base_idx + 4),
                            static_cast<uint32_t>(base_idx + 3),
                            static_cast<uint32_t>(base_idx + 4),
                            static_cast<uint32_t>(base_idx + 0)});
    }
  }
}

void OccupancyGrid::GenerateVoxelGeometry() {
  // For multi-layer voxel representation
  cell_vertices_.clear();
  cell_colors_.clear();
  cell_texcoords_.clear();
  cell_indices_.clear();

  // Process all layers
  for (size_t layer = 0; layer < layer_count_; ++layer) {
    // For voxel mode, all layer data should be in layer_data_ array
    if (layer >= layer_data_.size() || layer_data_[layer].empty()) continue;
    const auto& current_layer_data = layer_data_[layer];

    float layer_height = layer_heights_[layer];
    float layer_opacity = layer_opacities_[layer];

    for (size_t y = 0; y < height_; y += subsampling_factor_) {
      for (size_t x = 0; x < width_; x += subsampling_factor_) {
        float value = current_layer_data[y * width_ + x];
        if (!ShouldRenderCell(value)) continue;

        // Generate 3D voxel box for this cell
        GenerateVoxelCell(x, y, value, layer, layer_height, layer_opacity);
      }
    }
  }
}

void OccupancyGrid::GenerateContourGeometry() {
  // Generate contour lines based on height values
  GenerateGridGeometry();  // Base grid

  // TODO: Implement actual contour line generation
  // This would involve finding iso-lines at specific height values
  // For now, just use the basic grid geometry
}

void OccupancyGrid::GenerateQuadCell(size_t x, size_t y, float value,
                                     std::vector<glm::vec3>& vertices,
                                     std::vector<glm::vec3>& colors,
                                     std::vector<uint32_t>& indices) {
  glm::vec3 color = ComputeCellColor(value);
  glm::vec3 base = origin_ + glm::vec3(x * resolution_, y * resolution_, 0.0f);

  if (render_mode_ == RenderMode::kHeightmap) {
    base.z = ComputeCellHeight(value);
  }

  size_t base_idx = vertices.size();

  // Four corners of the quad
  vertices.push_back(base);
  vertices.push_back(base + glm::vec3(resolution_, 0.0f, 0.0f));
  vertices.push_back(base + glm::vec3(resolution_, resolution_, 0.0f));
  vertices.push_back(base + glm::vec3(0.0f, resolution_, 0.0f));

  // Colors
  for (int i = 0; i < 4; ++i) {
    colors.push_back(color);
    cell_texcoords_.push_back(glm::vec2(i % 2, i / 2));  // Basic UV mapping
  }

  // Two triangles
indices.insert(indices.end(),
    {static_cast<uint32_t>(base_idx + 0), static_cast<uint32_t>(base_idx + 1), static_cast<uint32_t>(base_idx + 2),
     static_cast<uint32_t>(base_idx + 0), static_cast<uint32_t>(base_idx + 2), static_cast<uint32_t>(base_idx + 3)});
}

void OccupancyGrid::GenerateCircleCell(size_t x, size_t y, float value,
                                       std::vector<glm::vec3>& vertices,
                                       std::vector<glm::vec3>& colors,
                                       std::vector<uint32_t>& indices) {
  glm::vec3 color = ComputeCellColor(value);
  glm::vec3 center = origin_ + glm::vec3((x + 0.5f) * resolution_,
                                         (y + 0.5f) * resolution_, 0.0f);

  if (render_mode_ == RenderMode::kHeightmap) {
    center.z = ComputeCellHeight(value);
  }

  size_t base_idx = vertices.size();
  float radius = resolution_ * 0.4f;  // Slightly smaller than cell size
  int segments = 8;                   // Octagon approximation

  // Center vertex
  vertices.push_back(center);
  colors.push_back(color);
  cell_texcoords_.push_back(glm::vec2(0.5f, 0.5f));

  // Ring vertices
  for (int i = 0; i < segments; ++i) {
    float angle =
        2.0f * M_PI * static_cast<float>(i) / static_cast<float>(segments);
    glm::vec3 offset =
        glm::vec3(std::cos(angle) * radius, std::sin(angle) * radius, 0.0f);
    vertices.push_back(center + offset);
    colors.push_back(color);
    cell_texcoords_.push_back(glm::vec2(0.5f + 0.5f * std::cos(angle),
                                        0.5f + 0.5f * std::sin(angle)));
  }

  // Generate triangles
  for (int i = 0; i < segments; ++i) {
    indices.insert(indices.end(),
                   {static_cast<uint32_t>(base_idx),
                    static_cast<uint32_t>(base_idx + 1 + i),
                    static_cast<uint32_t>(base_idx + 1 + (i + 1) % segments)});
  }
}

void OccupancyGrid::GenerateHexagonCell(size_t x, size_t y, float value,
                                        std::vector<glm::vec3>& vertices,
                                        std::vector<glm::vec3>& colors,
                                        std::vector<uint32_t>& indices) {
  glm::vec3 color = ComputeCellColor(value);
  glm::vec3 center = origin_ + glm::vec3((x + 0.5f) * resolution_,
                                         (y + 0.5f) * resolution_, 0.0f);

  // Offset every other row for hexagonal packing
  if (y % 2 == 1) {
    center.x += resolution_ * 0.5f;
  }

  if (render_mode_ == RenderMode::kHeightmap) {
    center.z = ComputeCellHeight(value);
  }

  size_t base_idx = vertices.size();
  float radius = resolution_ * 0.45f;

  // Center vertex
  vertices.push_back(center);
  colors.push_back(color);
  cell_texcoords_.push_back(glm::vec2(0.5f, 0.5f));

  // Six vertices of hexagon
  for (int i = 0; i < 6; ++i) {
    float angle = M_PI / 3.0f * static_cast<float>(i);
    glm::vec3 offset =
        glm::vec3(std::cos(angle) * radius, std::sin(angle) * radius, 0.0f);
    vertices.push_back(center + offset);
    colors.push_back(color);
    cell_texcoords_.push_back(glm::vec2(0.5f + 0.5f * std::cos(angle),
                                        0.5f + 0.5f * std::sin(angle)));
  }

  // Generate triangles
  for (int i = 0; i < 6; ++i) {
    indices.insert(indices.end(),
                   {static_cast<uint32_t>(base_idx),
                    static_cast<uint32_t>(base_idx + 1 + i),
                    static_cast<uint32_t>(base_idx + 1 + (i + 1) % 6)});
  }
}

void OccupancyGrid::GenerateVoxelCell(size_t x, size_t y, float value,
                                      size_t layer, float layer_height,
                                      float layer_opacity) {
  glm::vec3 color = ComputeCellColor(value, layer);
  color *= layer_opacity;  // Apply layer opacity

  glm::vec3 base =
      origin_ + glm::vec3(x * resolution_, y * resolution_, layer_height);
  float voxel_height = 0.3f;  // Fixed height for voxel layers

  size_t base_idx = cell_vertices_.size();

  // Generate a 3D box (8 vertices)
  // Bottom face
  cell_vertices_.push_back(base);
  cell_vertices_.push_back(base + glm::vec3(resolution_, 0.0f, 0.0f));
  cell_vertices_.push_back(base + glm::vec3(resolution_, resolution_, 0.0f));
  cell_vertices_.push_back(base + glm::vec3(0.0f, resolution_, 0.0f));

  // Top face
  cell_vertices_.push_back(base + glm::vec3(0.0f, 0.0f, voxel_height));
  cell_vertices_.push_back(base + glm::vec3(resolution_, 0.0f, voxel_height));
  cell_vertices_.push_back(base +
                           glm::vec3(resolution_, resolution_, voxel_height));
  cell_vertices_.push_back(base + glm::vec3(0.0f, resolution_, voxel_height));

  // Colors for all 8 vertices
  for (int i = 0; i < 8; ++i) {
    cell_colors_.push_back(color);
    cell_texcoords_.push_back(
        glm::vec2(i % 2, (i / 2) % 2));  // Basic UV mapping
  }

  // Generate indices for the 6 faces of the box (12 triangles)
  // Bottom face (facing down)
  cell_indices_.insert(
      cell_indices_.end(),
      {static_cast<uint32_t>(base_idx + 0), static_cast<uint32_t>(base_idx + 2),
       static_cast<uint32_t>(base_idx + 1), static_cast<uint32_t>(base_idx + 0),
       static_cast<uint32_t>(base_idx + 3),
       static_cast<uint32_t>(base_idx + 2)});

  // Top face (facing up)
  cell_indices_.insert(
      cell_indices_.end(),
      {static_cast<uint32_t>(base_idx + 4), static_cast<uint32_t>(base_idx + 5),
       static_cast<uint32_t>(base_idx + 6), static_cast<uint32_t>(base_idx + 4),
       static_cast<uint32_t>(base_idx + 6),
       static_cast<uint32_t>(base_idx + 7)});

  // Front face
  cell_indices_.insert(
      cell_indices_.end(),
      {static_cast<uint32_t>(base_idx + 0), static_cast<uint32_t>(base_idx + 1),
       static_cast<uint32_t>(base_idx + 5), static_cast<uint32_t>(base_idx + 0),
       static_cast<uint32_t>(base_idx + 5),
       static_cast<uint32_t>(base_idx + 4)});

  // Back face
  cell_indices_.insert(
      cell_indices_.end(),
      {static_cast<uint32_t>(base_idx + 2), static_cast<uint32_t>(base_idx + 7),
       static_cast<uint32_t>(base_idx + 6), static_cast<uint32_t>(base_idx + 2),
       static_cast<uint32_t>(base_idx + 3),
       static_cast<uint32_t>(base_idx + 7)});

  // Left face
  cell_indices_.insert(
      cell_indices_.end(),
      {static_cast<uint32_t>(base_idx + 0), static_cast<uint32_t>(base_idx + 4),
       static_cast<uint32_t>(base_idx + 7), static_cast<uint32_t>(base_idx + 0),
       static_cast<uint32_t>(base_idx + 7),
       static_cast<uint32_t>(base_idx + 3)});

  // Right face
  cell_indices_.insert(
      cell_indices_.end(),
      {static_cast<uint32_t>(base_idx + 1), static_cast<uint32_t>(base_idx + 2),
       static_cast<uint32_t>(base_idx + 6), static_cast<uint32_t>(base_idx + 1),
       static_cast<uint32_t>(base_idx + 6),
       static_cast<uint32_t>(base_idx + 5)});
}

glm::vec3 OccupancyGrid::ComputeCellColor(float value, size_t layer) const {
  if (value < 0.0f) {
    return unknown_color_;
  }

  switch (color_mode_) {
    case ColorMode::kOccupancy:
      if (value < 0.3f)
        return free_color_;
      else if (value > 0.7f)
        return occupied_color_;
      else
        return glm::mix(free_color_, occupied_color_, (value - 0.3f) / 0.4f);

    case ColorMode::kProbability:
      return glm::mix(glm::vec3(0.0f), glm::vec3(1.0f), value);

    case ColorMode::kCostmap:
      // Blue to red gradient
      return glm::mix(glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f),
                      value);

    case ColorMode::kHeight:
      // Rainbow gradient
      if (value < 0.2f)
        return glm::mix(glm::vec3(0.0f, 0.0f, 1.0f),
                        glm::vec3(0.0f, 1.0f, 1.0f), value * 5.0f);
      else if (value < 0.4f)
        return glm::mix(glm::vec3(0.0f, 1.0f, 1.0f),
                        glm::vec3(0.0f, 1.0f, 0.0f), (value - 0.2f) * 5.0f);
      else if (value < 0.6f)
        return glm::mix(glm::vec3(0.0f, 1.0f, 0.0f),
                        glm::vec3(1.0f, 1.0f, 0.0f), (value - 0.4f) * 5.0f);
      else if (value < 0.8f)
        return glm::mix(glm::vec3(1.0f, 1.0f, 0.0f),
                        glm::vec3(1.0f, 0.0f, 0.0f), (value - 0.6f) * 5.0f);
      else
        return glm::mix(glm::vec3(1.0f, 0.0f, 0.0f),
                        glm::vec3(1.0f, 0.0f, 1.0f), (value - 0.8f) * 5.0f);

    case ColorMode::kSemantic:
      // Use layer-based coloring
      if (layer < custom_colors_.size()) {
        return custom_colors_[layer];
      }
      return unknown_color_;

    case ColorMode::kCustom:
      if (!custom_colors_.empty()) {
        size_t index = static_cast<size_t>(value * (custom_colors_.size() - 1));
        index = std::min(index, custom_colors_.size() - 1);
        return custom_colors_[index];
      }
      return unknown_color_;

    default:
      return unknown_color_;
  }
}

float OccupancyGrid::ComputeCellHeight(float value) const {
  if (value < 0.0f) return 0.0f;

  float height = value * height_scale_;
  return glm::clamp(height, 0.0f, max_height_);
}

bool OccupancyGrid::ShouldRenderCell(float value) const {
  // Don't render unknown cells (negative values)
  if (value < 0.0f) {
    return false;
  }

  // For voxel mode, only render occupied cells (not free space)
  if (render_mode_ == RenderMode::kVoxels) {
    // Default threshold for voxel mode is 0.5 (occupied)
    float threshold = (value_threshold_ > 0.0f) ? value_threshold_ : 0.5f;
    return value >= threshold;
  }

  // Apply value threshold if set (but always render 0.0f as free space for 2D
  // modes)
  if (value_threshold_ > 0.0f && value > 0.0f && value < value_threshold_) {
    return false;
  }

  return true;
}

void OccupancyGrid::UpdateGpuBuffers() {
  if (cell_vertices_.empty()) return;

  glBindVertexArray(vao_grid_);

  // Upload vertices
  glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices_);
  glBufferData(GL_ARRAY_BUFFER, cell_vertices_.size() * sizeof(glm::vec3),
               cell_vertices_.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(0);

  // Upload colors
  glBindBuffer(GL_ARRAY_BUFFER, vbo_colors_);
  glBufferData(GL_ARRAY_BUFFER, cell_colors_.size() * sizeof(glm::vec3),
               cell_colors_.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
  glEnableVertexAttribArray(1);

  // Upload texture coordinates
  glBindBuffer(GL_ARRAY_BUFFER, vbo_texcoords_);
  glBufferData(GL_ARRAY_BUFFER, cell_texcoords_.size() * sizeof(glm::vec2),
               cell_texcoords_.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
  glEnableVertexAttribArray(2);

  // Upload indices
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_indices_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, cell_indices_.size() * sizeof(uint32_t),
               cell_indices_.data(), GL_STATIC_DRAW);

  glBindVertexArray(0);
}

void OccupancyGrid::UpdateGridBuffers() {
  if (!show_grid_) return;

  grid_vertices_.clear();

  // Horizontal lines
  for (size_t y = 0; y <= height_; ++y) {
    glm::vec3 start = origin_ + glm::vec3(0.0f, y * resolution_, 0.01f);
    glm::vec3 end =
        origin_ + glm::vec3(width_ * resolution_, y * resolution_, 0.01f);
    grid_vertices_.push_back(start);
    grid_vertices_.push_back(end);
  }

  // Vertical lines
  for (size_t x = 0; x <= width_; ++x) {
    glm::vec3 start = origin_ + glm::vec3(x * resolution_, 0.0f, 0.01f);
    glm::vec3 end =
        origin_ + glm::vec3(x * resolution_, height_ * resolution_, 0.01f);
    grid_vertices_.push_back(start);
    grid_vertices_.push_back(end);
  }

  if (!grid_vertices_.empty()) {
    glBindBuffer(GL_ARRAY_BUFFER, vbo_grid_lines_);
    glBufferData(GL_ARRAY_BUFFER, grid_vertices_.size() * sizeof(glm::vec3),
                 grid_vertices_.data(), GL_STATIC_DRAW);
  }
}

void OccupancyGrid::UpdateBorderBuffers() {
  if (border_width_ <= 0.0f) return;

  border_vertices_.clear();

  // Four corners of the grid boundary
  glm::vec3 corner1 = origin_ + glm::vec3(0.0f, 0.0f, 0.02f);
  glm::vec3 corner2 = origin_ + glm::vec3(width_ * resolution_, 0.0f, 0.02f);
  glm::vec3 corner3 =
      origin_ + glm::vec3(width_ * resolution_, height_ * resolution_, 0.02f);
  glm::vec3 corner4 = origin_ + glm::vec3(0.0f, height_ * resolution_, 0.02f);

  border_vertices_.push_back(corner1);
  border_vertices_.push_back(corner2);
  border_vertices_.push_back(corner3);
  border_vertices_.push_back(corner4);

  if (!border_vertices_.empty()) {
    glBindBuffer(GL_ARRAY_BUFFER, vbo_border_lines_);
    glBufferData(GL_ARRAY_BUFFER, border_vertices_.size() * sizeof(glm::vec3),
                 border_vertices_.data(), GL_STATIC_DRAW);
  }
}

}  // namespace quickviz