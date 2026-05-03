/*
 * @file demo.cpp
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "core/demo.hpp"

#include <cmath>
#include <random>

namespace quickviz::demo {

namespace {

// HSV → RGB with H in [0,1), S = V = 1.
glm::vec3 HueToRgb(float h) {
  const float r = std::abs(h * 6.0f - 3.0f) - 1.0f;
  const float g = 2.0f - std::abs(h * 6.0f - 2.0f);
  const float b = 2.0f - std::abs(h * 6.0f - 4.0f);
  return glm::clamp(glm::vec3{r, g, b}, 0.0f, 1.0f);
}

constexpr float kPi = 3.14159265358979323846f;

}  // namespace

PointCloudData SpiralCloud(std::size_t num_points,
                           float radius,
                           float height,
                           float turns) {
  PointCloudData out;
  if (num_points == 0) return out;

  out.points.reserve(num_points);
  out.colors.reserve(num_points);

  const float denom = static_cast<float>(num_points - 1);
  for (std::size_t i = 0; i < num_points; ++i) {
    const float t = (num_points == 1) ? 0.0f : static_cast<float>(i) / denom;
    const float angle = t * 2.0f * kPi * turns;
    const float r = t * radius;
    const float z = (t - 0.5f) * height;
    out.points.emplace_back(r * std::cos(angle), r * std::sin(angle), z);
    out.colors.push_back(HueToRgb(t));
  }
  return out;
}

PointCloudData PlanarPointGrid(std::size_t rows,
                               std::size_t cols,
                               float spacing) {
  PointCloudData out;
  if (rows == 0 || cols == 0) return out;

  const std::size_t n = rows * cols;
  out.points.reserve(n);
  out.colors.reserve(n);

  const float row_denom = (rows == 1) ? 1.0f : static_cast<float>(rows - 1);
  const float col_denom = (cols == 1) ? 1.0f : static_cast<float>(cols - 1);
  const float origin_x = -0.5f * static_cast<float>(cols - 1) * spacing;
  const float origin_y = -0.5f * static_cast<float>(rows - 1) * spacing;

  for (std::size_t r = 0; r < rows; ++r) {
    for (std::size_t c = 0; c < cols; ++c) {
      out.points.emplace_back(origin_x + static_cast<float>(c) * spacing,
                              origin_y + static_cast<float>(r) * spacing,
                              0.0f);
      const float t =
          0.5f * (static_cast<float>(r) / row_denom +
                  static_cast<float>(c) / col_denom);
      out.colors.push_back(HueToRgb(t));
    }
  }
  return out;
}

PointCloudData NoiseCloud(std::size_t num_points, float sigma, unsigned seed) {
  PointCloudData out;
  if (num_points == 0) return out;

  std::mt19937 rng(seed == 0 ? 0xC0FFEEu : seed);
  std::normal_distribution<float> dist(0.0f, sigma);

  out.points.reserve(num_points);
  out.colors.reserve(num_points);
  for (std::size_t i = 0; i < num_points; ++i) {
    out.points.emplace_back(dist(rng), dist(rng), dist(rng));
    // Color encodes distance from origin so denser clusters read visually.
    const float d =
        glm::length(out.points.back()) / (3.0f * std::max(sigma, 1e-6f));
    out.colors.push_back(HueToRgb(glm::clamp(d, 0.0f, 1.0f)));
  }
  return out;
}

MeshData CubeMesh(glm::vec3 center, float size) {
  MeshData out;
  const float h = 0.5f * size;

  out.vertices = {
      // 0..3: -Z face
      center + glm::vec3{-h, -h, -h}, center + glm::vec3{ h, -h, -h},
      center + glm::vec3{ h,  h, -h}, center + glm::vec3{-h,  h, -h},
      // 4..7: +Z face
      center + glm::vec3{-h, -h,  h}, center + glm::vec3{ h, -h,  h},
      center + glm::vec3{ h,  h,  h}, center + glm::vec3{-h,  h,  h},
  };

  // 12 triangles, CCW when viewed from outside.
  out.indices = {
      0, 2, 1,  0, 3, 2,  // -Z
      4, 5, 6,  4, 6, 7,  // +Z
      0, 1, 5,  0, 5, 4,  // -Y
      1, 2, 6,  1, 6, 5,  // +X
      2, 3, 7,  2, 7, 6,  // +Y
      3, 0, 4,  3, 4, 7,  // -X
  };
  return out;
}

std::vector<glm::vec3> Trajectory(std::size_t num_points, float scale) {
  std::vector<glm::vec3> out;
  if (num_points == 0) return out;

  out.reserve(num_points);
  const float denom = (num_points == 1) ? 1.0f
                                        : static_cast<float>(num_points - 1);
  for (std::size_t i = 0; i < num_points; ++i) {
    const float t = static_cast<float>(i) / denom;
    const float angle = t * 2.0f * kPi;
    // 3:2:1 Lissajous in 3D.
    out.emplace_back(scale * std::sin(3.0f * angle),
                     scale * std::sin(2.0f * angle),
                     0.5f * scale * std::sin(angle));
  }
  return out;
}

}  // namespace quickviz::demo
