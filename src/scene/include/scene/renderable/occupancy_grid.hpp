/**
 * @file occupancy_grid.hpp
 * @brief 2D occupancy-grid renderable
 *
 * Renders a 2D probabilistic occupancy grid as a textured quad lying in
 * the XY plane (Z = 0 in the local frame). Designed to consume ROS-style
 * data without lossy conversion: cells are int8_t in the range
 * `[-1, 100]`, where `-1` is unknown and `0..100` is the probability
 * (percent) of occupancy.
 *
 * The renderable does not interpret or transform the grid origin
 * pose-wise — pass world-space `origin` and `resolution`, or use the
 * `SetTransform` machinery on `OpenGlObject` for richer placement.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_OCCUPANCY_GRID_HPP
#define QUICKVIZ_OCCUPANCY_GRID_HPP

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "scene/interface/opengl_object.hpp"
#include "../shader_program.hpp"

namespace quickviz {

class OccupancyGrid : public OpenGlObject {
 public:
  OccupancyGrid();
  ~OccupancyGrid();

  // === Data ===

  /**
   * @brief Set the grid contents.
   *
   * @param width      Number of cells along the X axis.
   * @param height     Number of cells along the Y axis.
   * @param resolution World-space size of a single cell, in meters.
   * @param origin     World-space position of cell `(0, 0)`. The grid
   *                   spans from `origin` to
   *                   `origin + (width, height) * resolution` in XY.
   * @param values     Row-major occupancy values. Size must be
   *                   `width * height`. ROS convention:
   *                     -1     → unknown
   *                      0..100 → probability of occupancy (percent)
   *
   * Safe to call repeatedly; the texture is updated in place when the
   * dimensions don't change. Grid resizes do reallocate the texture.
   * Must be called on the render thread.
   */
  void SetGrid(uint32_t width, uint32_t height, float resolution,
               const glm::vec3& origin, const std::vector<int8_t>& values);

  // === Appearance ===

  /// Color used for cells with occupancy = 0 (free). Default: light gray.
  void SetFreeColor(const glm::vec4& color) { free_color_ = color; }
  /// Color used for cells with occupancy = 100 (occupied). Default: black.
  void SetOccupiedColor(const glm::vec4& color) { occupied_color_ = color; }
  /// Color used for cells flagged unknown (-1). Default: medium gray.
  void SetUnknownColor(const glm::vec4& color) { unknown_color_ = color; }

  // === Inspection ===

  uint32_t GetWidth() const { return width_; }
  uint32_t GetHeight() const { return height_; }
  float GetResolution() const { return resolution_; }
  const glm::vec3& GetOrigin() const { return origin_; }

  // === OpenGlObject interface ===

  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override {
    return vao_ != 0;
  }

 private:
  void EnsureTextureSize(uint32_t width, uint32_t height);
  void UploadTexel(const std::vector<int8_t>& values);

  // Grid metadata
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  float resolution_ = 1.0f;
  glm::vec3 origin_ = glm::vec3(0.0f);

  // Color configuration (sane defaults).
  glm::vec4 free_color_ = glm::vec4(0.85f, 0.85f, 0.85f, 1.0f);
  glm::vec4 occupied_color_ = glm::vec4(0.05f, 0.05f, 0.05f, 1.0f);
  glm::vec4 unknown_color_ = glm::vec4(0.45f, 0.45f, 0.45f, 1.0f);

  // GL resources
  uint32_t vao_ = 0;
  uint32_t vbo_ = 0;
  uint32_t texture_id_ = 0;
  ShaderProgram shader_;
};

}  // namespace quickviz

#endif  // QUICKVIZ_OCCUPANCY_GRID_HPP
