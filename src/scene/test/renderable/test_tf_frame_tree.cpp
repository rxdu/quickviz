/**
 * @file test_tf_frame_tree.cpp
 * @brief Visual test for TfFrameTree
 *
 * Builds a small robot-like kinematics tree and animates the joints
 * over time so the parent-child relationships are visible. Run and
 * inspect:
 *
 *   - Six axis triplets (RGB) at:
 *       world (origin), base, lidar, arm_base, arm_link_1, arm_tip
 *   - Gray lines connecting parents to children, showing tree topology
 *   - Smooth animation of the arm and base over time
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "scene/renderable/grid.hpp"
#include "scene/renderable/tf_frame_tree.hpp"
#include "scene/scene_app.hpp"

using namespace quickviz;

namespace {

glm::mat4 Translation(const glm::vec3& t) {
  return glm::translate(glm::mat4(1.0f), t);
}

glm::mat4 RotZ(float angle) {
  return glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, 0.0f, 1.0f));
}

}  // namespace

int main() {
  try {
    SceneApp::Config config;
    config.window_title = "TfFrameTree Rendering Test";
    SceneApp view(config);

    view.SetDescription(
        "Six-frame robot kinematics tree with animated joints. Each "
        "frame draws as RGB axes; gray lines connect parents to children.");

    // Capture by reference into the lambda; SceneApp keeps it alive.
    TfFrameTree* tree_ptr = nullptr;

    view.SetSceneSetup([&tree_ptr](SceneManager* scene) {
      auto reference = std::make_unique<Grid>(8.0f, 0.5f,
                                              glm::vec3(0.6f, 0.6f, 0.6f));
      scene->AddOpenGLObject("grid", std::move(reference));

      auto tree = std::make_unique<TfFrameTree>(/*axis_length=*/0.4f);

      // Static parts of the tree.
      tree->SetFrame("world", "", glm::mat4(1.0f));
      tree->SetFrame("base", "world",
                     Translation(glm::vec3(1.0f, 1.0f, 0.0f)) * RotZ(0.0f));
      tree->SetFrame("lidar", "base",
                     Translation(glm::vec3(0.3f, 0.0f, 0.4f)));
      tree->SetFrame("arm_base", "base",
                     Translation(glm::vec3(0.0f, 0.0f, 0.5f)));
      tree->SetFrame("arm_link_1", "arm_base",
                     Translation(glm::vec3(0.5f, 0.0f, 0.0f)));
      tree->SetFrame("arm_tip", "arm_link_1",
                     Translation(glm::vec3(0.5f, 0.0f, 0.0f)));

      tree_ptr = tree.get();
      scene->AddOpenGLObject("tf_tree", std::move(tree));

      // Animate base yaw + arm joint angles each frame.
      const auto t0 = std::chrono::steady_clock::now();
      scene->SetPreDrawCallback([&tree_ptr, t0]() {
        if (!tree_ptr) return;
        const float t = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - t0).count();

        // Base spins slowly.
        tree_ptr->SetFrame(
            "base", "world",
            Translation(glm::vec3(1.0f, 1.0f, 0.0f)) * RotZ(0.3f * t));

        // Arm shoulder oscillates.
        tree_ptr->SetFrame(
            "arm_link_1", "arm_base",
            RotZ(0.6f * std::sin(1.2f * t)) *
                Translation(glm::vec3(0.5f, 0.0f, 0.0f)));

        // Arm tip rotates a bit too — grandchildren update without
        // restating the chain because GetWorldTransform walks parents.
        tree_ptr->SetFrame(
            "arm_tip", "arm_link_1",
            RotZ(0.8f * std::cos(1.5f * t)) *
                Translation(glm::vec3(0.5f, 0.0f, 0.0f)));
      });
    });

    view.Run();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
