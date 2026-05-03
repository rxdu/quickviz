/*
 * @file test_demo.cpp
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cmath>

#include <gtest/gtest.h>

#include "core/demo.hpp"

namespace quickviz::demo {
namespace {

TEST(DemoSpiralCloud, SizesMatchAndAreNonEmpty) {
  auto result = SpiralCloud(/*num_points=*/100);
  EXPECT_EQ(result.points.size(), 100u);
  EXPECT_EQ(result.colors.size(), 100u);
}

TEST(DemoSpiralCloud, ZeroPointsReturnsEmpty) {
  auto result = SpiralCloud(0);
  EXPECT_TRUE(result.points.empty());
  EXPECT_TRUE(result.colors.empty());
}

TEST(DemoSpiralCloud, FillsRequestedHeightRange) {
  constexpr float kHeight = 4.0f;
  auto result = SpiralCloud(64, /*radius=*/1.0f, kHeight, /*turns=*/2.0f);
  ASSERT_FALSE(result.points.empty());
  EXPECT_NEAR(result.points.front().z, -kHeight * 0.5f, 1e-5f);
  EXPECT_NEAR(result.points.back().z, kHeight * 0.5f, 1e-5f);
}

TEST(DemoPlanarPointGrid, GridShape) {
  auto result = PlanarPointGrid(3, 4, 0.1f);
  EXPECT_EQ(result.points.size(), 12u);
  EXPECT_EQ(result.colors.size(), 12u);
  // Every point lies in the Z=0 plane.
  for (const auto& p : result.points) EXPECT_FLOAT_EQ(p.z, 0.0f);
}

TEST(DemoNoiseCloud, IsDeterministicForSameSeed) {
  auto a = NoiseCloud(50, 1.0f, /*seed=*/42);
  auto b = NoiseCloud(50, 1.0f, /*seed=*/42);
  ASSERT_EQ(a.points.size(), b.points.size());
  for (std::size_t i = 0; i < a.points.size(); ++i) {
    EXPECT_FLOAT_EQ(a.points[i].x, b.points[i].x);
    EXPECT_FLOAT_EQ(a.points[i].y, b.points[i].y);
    EXPECT_FLOAT_EQ(a.points[i].z, b.points[i].z);
  }
}

TEST(DemoCubeMesh, HasEightVerticesAndTwelveTriangles) {
  auto mesh = CubeMesh();
  EXPECT_EQ(mesh.vertices.size(), 8u);
  EXPECT_EQ(mesh.indices.size(), 36u);  // 12 triangles × 3 indices
  for (auto idx : mesh.indices) EXPECT_LT(idx, 8u);
}

TEST(DemoCubeMesh, RespectsCenterAndSize) {
  auto mesh = CubeMesh(glm::vec3{1.0f, 2.0f, 3.0f}, 2.0f);
  // All vertices should sit on the bounding box of the requested cube.
  for (const auto& v : mesh.vertices) {
    EXPECT_NEAR(std::abs(v.x - 1.0f), 1.0f, 1e-5f);
    EXPECT_NEAR(std::abs(v.y - 2.0f), 1.0f, 1e-5f);
    EXPECT_NEAR(std::abs(v.z - 3.0f), 1.0f, 1e-5f);
  }
}

TEST(DemoTrajectory, HasRequestedSizeAndIsBounded) {
  constexpr float kScale = 5.0f;
  auto traj = Trajectory(200, kScale);
  EXPECT_EQ(traj.size(), 200u);
  for (const auto& p : traj) {
    EXPECT_LE(std::abs(p.x), kScale + 1e-4f);
    EXPECT_LE(std::abs(p.y), kScale + 1e-4f);
    EXPECT_LE(std::abs(p.z), 0.5f * kScale + 1e-4f);
  }
}

}  // namespace
}  // namespace quickviz::demo
