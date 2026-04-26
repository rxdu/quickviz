/**
 * @file test_occupancy_grid.cpp
 * @brief Visual test for the OccupancyGrid renderable
 *
 * Renders a 50x50 occupancy grid with a hand-crafted pattern that
 * exercises all three cell states (free, occupied, unknown). Run and
 * inspect:
 *
 *   - Outer frame of black (occupied) cells.
 *   - Inner light-gray (free) field with a diagonal corridor of
 *     intermediate occupancy values.
 *   - A square patch of unknown (medium gray) cells in the lower-left.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <cstdint>
#include <iostream>
#include <vector>

#include <glm/glm.hpp>

#include "scene/renderable/grid.hpp"
#include "scene/renderable/occupancy_grid.hpp"
#include "scene/scene_app.hpp"

using namespace quickviz;

namespace {

constexpr uint32_t kWidth = 50;
constexpr uint32_t kHeight = 50;
constexpr float kResolution = 0.2f;  // 0.2m per cell → 10m × 10m grid

std::vector<int8_t> BuildPattern() {
  std::vector<int8_t> data(kWidth * kHeight, 0);  // free everywhere

  for (uint32_t y = 0; y < kHeight; ++y) {
    for (uint32_t x = 0; x < kWidth; ++x) {
      const std::size_t idx = y * kWidth + x;

      // Outer one-cell border: occupied.
      if (x == 0 || y == 0 || x == kWidth - 1 || y == kHeight - 1) {
        data[idx] = 100;
      }

      // Diagonal corridor with intermediate occupancy values.
      if (x == y) {
        data[idx] = 50;
      }

      // Lower-left square of unknown cells.
      if (x < 12 && y < 12 && (x > 0 && y > 0)) {
        data[idx] = -1;
      }
    }
  }
  return data;
}

}  // namespace

void Setup(SceneManager* scene) {
  // Reference grid at the same resolution / extent so the cells line up
  // with the world axes for easy visual verification.
  auto reference = std::make_unique<Grid>(
      static_cast<float>(kWidth) * kResolution, kResolution,
      glm::vec3(0.6f, 0.6f, 0.6f));
  scene->AddOpenGLObject("grid", std::move(reference));

  auto og = std::make_unique<OccupancyGrid>();
  // Origin chosen so the centre of the grid sits at world origin.
  const float half_w = 0.5f * static_cast<float>(kWidth) * kResolution;
  const float half_h = 0.5f * static_cast<float>(kHeight) * kResolution;
  og->SetGrid(kWidth, kHeight, kResolution,
              glm::vec3(-half_w, -half_h, 0.0f), BuildPattern());
  scene->AddOpenGLObject("occupancy_grid", std::move(og));
}

int main() {
  try {
    SceneApp::Config config;
    config.window_title = "OccupancyGrid Rendering Test";
    config.show_grid = false;  // we add our own at the matching resolution

    SceneApp view(config);
    view.SetDescription(
        "OccupancyGrid: 50x50 cells at 0.2m resolution. Outer frame "
        "occupied (black), interior free (light gray), diagonal "
        "intermediate, lower-left corner unknown (medium gray).");
    view.SetSceneSetup(Setup);
    view.Run();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
