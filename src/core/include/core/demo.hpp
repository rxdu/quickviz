/*
 * @file demo.hpp
 * @brief Synthetic data generators for quickstarts, tutorials, and tests
 *
 * `quickviz::demo` provides small, deterministic data generators so users
 * can explore the library without supplying their own data files. Returned
 * structures match the shapes expected by the library's renderables — pass
 * a `PointCloudData` straight into `PointCloud::SetPoints`, a `MeshData`
 * straight into `Mesh::SetVertices`/`SetIndices`.
 *
 * All generators are pure functions of their inputs: no global state, no
 * I/O, deterministic for the seeds you provide.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CORE_DEMO_HPP
#define QUICKVIZ_CORE_DEMO_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace quickviz::demo {

/**
 * @brief Result of a point-cloud generator: positions + per-point RGB.
 *
 * `points` and `colors` are always the same size. Pass directly into
 * `PointCloud::SetPoints(points, colors)`.
 */
struct PointCloudData {
  std::vector<glm::vec3> points;
  std::vector<glm::vec3> colors;
};

/**
 * @brief Result of a mesh generator: vertex positions + triangle indices.
 *
 * Pass `vertices` to `Mesh::SetVertices` and `indices` to `Mesh::SetIndices`.
 * Indices are 32-bit unsigned for direct compatibility with the GL element
 * buffer format used by the `Mesh` renderable.
 */
struct MeshData {
  std::vector<glm::vec3> vertices;
  std::vector<uint32_t> indices;
};

/**
 * @brief Generate a vertical helix of coloured points.
 *
 * Useful as the canonical "hello, world" point cloud — visually
 * distinctive, deterministic, and small. Color is HSV-derived from the
 * height parameter so the result is colorful without needing extra
 * configuration.
 *
 * @param num_points Number of points along the helix (≥ 1).
 * @param radius     Maximum spiral radius in world units.
 * @param height     Total vertical extent (the spiral spans ±height/2).
 * @param turns      How many full revolutions across the height.
 */
PointCloudData SpiralCloud(std::size_t num_points,
                           float radius = 3.0f,
                           float height = 6.0f,
                           float turns = 8.0f);

/**
 * @brief Generate a regular grid of points lying in the Z=0 plane.
 *
 * Colors interpolate diagonally so adjacent points are distinguishable.
 *
 * @param rows    Number of grid rows (>= 1).
 * @param cols    Number of grid columns (>= 1).
 * @param spacing Distance between neighbouring points in world units.
 */
PointCloudData PlanarPointGrid(std::size_t rows,
                               std::size_t cols,
                               float spacing = 0.1f);

/**
 * @brief Generate a Gaussian-noise point cloud centred on the origin.
 *
 * @param num_points Number of points.
 * @param sigma      Standard deviation along each axis in world units.
 * @param seed       Seed for reproducibility (0 = use a fixed default).
 */
PointCloudData NoiseCloud(std::size_t num_points,
                          float sigma = 1.0f,
                          unsigned seed = 0);

/**
 * @brief Generate a unit cube mesh centred on `center` with the given size.
 *
 * 8 vertices, 12 triangles (36 indices). Suitable as a basic Mesh demo.
 */
MeshData CubeMesh(glm::vec3 center = glm::vec3(0.0f), float size = 1.0f);

/**
 * @brief Generate a smooth synthetic 3D trajectory.
 *
 * The trajectory is a Lissajous-like curve in 3D — useful for showing
 * line strips and path renderables without needing real robot data.
 *
 * @param num_points Number of samples along the trajectory.
 * @param scale      Overall extent in world units.
 */
std::vector<glm::vec3> Trajectory(std::size_t num_points,
                                  float scale = 5.0f);

}  // namespace quickviz::demo

#endif  // QUICKVIZ_CORE_DEMO_HPP
